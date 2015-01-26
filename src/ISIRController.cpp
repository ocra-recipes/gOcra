/**
 * \file ISIRController.cpp
 * \author Joseph Salini
 *
 * \brief Implement the LQP-based controller developped during my PhD thesis with xde framework.
 *
 * File history
 * - 14/06/13: Mingxing Liu - Adaptation to generalized smooth hierarchical controller (GSHC).
 */

#include "orcisir/ISIRController.h"

#include <iostream>

#include "orc/optim/QuadraticFunction.h"

#include "orcisir/Solvers/ISIRSolver.h"
#include "orcisir/Tasks/ISIRTask.h"

#include "orcisir/Performances.h"
#include "orcisir/OrthonormalFamily.h"

namespace orcisir
{

class FcQuadraticFunction : public orc::QuadraticFunction
{
public:

    FcQuadraticFunction(orc::Variable& x)
    : NamedInstance("Variable Fc Quadratic Function")
    , orc::AbilitySet(orc::PARTIAL_X, orc::PARTIAL_XX)
    , CoupledInputOutputSize(false)
    , QuadraticFunction(x)
    {

    }

    virtual ~FcQuadraticFunction() {};

    void doUpdateInputSizeBegin() {};

    void updateHessian() const
    {
        IFunction<orc::PARTIAL_XX>::_val[0]->setIdentity(x.getSize(), x.getSize());
    }

    void updateq() const
    {
        _q[0]->setZero(x.getSize());
    }

    void updater() const
    {
        _r[0] = 0;
    }

};


struct ISIRController::Pimpl
{
    Model&       innerModel;
    ISIRSolver&  innerSolver;
    bool         reducedProblem;
    bool         useGSHC;
    bool         GSHCConstraintIsActive;
    CompositeVariable var;

    // EQUALITY CONSTRAINT OF THE DYNAMIC EQUATION
    orc::EqualZeroConstraintPtr< ISIRDynamicFunction >      dynamicEquation;

    // MINIMIZATION TASK FOR WHOLE VARIABLE MINIMIZATION
    orc::QuadraticFunction*     minDdqFunction;
    orc::QuadraticFunction*     minTauFunction;
    FcQuadraticFunction*        minFcFunction;

    ObjectivePtr<orc::QuadraticFunction>     minDdqObjective;
    ObjectivePtr<orc::QuadraticFunction>     minTauObjective;
    ObjectivePtr<orc::QuadraticFunction>     minFcObjective;

    std::vector< ISIRTask* >                 createdTask;
    std::vector< ISIRTask* >                 activeTask;
    int                                      nbActiveTask;
    int                                      totalActiveTaskDim;
    MatrixXd                                 augmentedJacobian;
    std::vector< MatrixXd >                  priorityMatrix;

    // PERFORMANCE RECORDERS
    PerformanceRecorder updateTasksRecorder;
    PerformanceRecorder solveProblemRecorder;

    Pimpl(Model& m, ISIRSolver&  s, bool useReducedProblem, bool useGSHC)
        : innerModel(m)
        , innerSolver(s)
        , reducedProblem(useReducedProblem)
        , useGSHC(useGSHC)
        , GSHCConstraintIsActive(false)
        , var("taskVariables.var",m.getAccelerationVariable())
        , dynamicEquation( new ISIRDynamicFunction(m) )
        , minDdqFunction( new orc::QuadraticFunction(m.getAccelerationVariable(), Eigen::MatrixXd::Identity(m.nbDofs(), m.nbDofs()), Eigen::VectorXd::Zero(m.nbDofs()), 0) )
        , minTauFunction( new orc::QuadraticFunction(m.getJointTorqueVariable(), Eigen::MatrixXd::Identity(m.getJointTorqueVariable().getSize(),m.getJointTorqueVariable().getSize()), Eigen::VectorXd::Zero(m.getJointTorqueVariable().getSize()), 0) )
        , minFcFunction( new FcQuadraticFunction(m.getModelContacts().getContactForcesVariable()) )
        , nbActiveTask(0)
        , totalActiveTaskDim(0)
        , augmentedJacobian(MatrixXd::Zero(1,m.nbDofs()))
    {
        minDdqObjective.set(minDdqFunction);
        minTauObjective.set(minTauFunction);
        minFcObjective.set(minFcFunction);
//        createdTask.reserve(20);
//        activeTask.reserve(20);
    }

    ~Pimpl()
    {

    }
};



/** Initialize ISIR controller.
 *
 * \param ctrlName The name of the controller
 * \param innerModel The internal model of the robot one wants to control
 * \param innerSolver The internal solver one wants to use to make the quadratic optimization
 * \param useReducedProblem Tell if the redundant problem is considered (unknown variable is \f$ [ \ddq \; \torque \; \force_c ] \f$),
 *        or is the reduced problem (non-redundant) is considred (unknown variable is \f$ [ \torque \; \force_c ] \f$)
 */
ISIRController::ISIRController(const std::string& ctrlName, Model& innerModel, ISIRSolver& innerSolver, bool useReducedProblem, bool useGSHC)
    : Controller(ctrlName, innerModel)
    , pimpl( new Pimpl(innerModel, innerSolver, useReducedProblem, useGSHC) )
{
    if (!pimpl->reducedProblem)
    {
        pimpl->innerSolver.addConstraint(pimpl->dynamicEquation.getConstraint());
        pimpl->innerSolver.addObjective(pimpl->minDdqObjective);
        pimpl->innerSolver.addObjective(pimpl->minTauObjective);
        pimpl->innerSolver.addObjective(pimpl->minFcObjective);
    }
    else
    {
        pimpl->innerSolver.addObjective(pimpl->minTauObjective);
        pimpl->innerSolver.addObjective(pimpl->minFcObjective);
    }
    setVariableMinimizationWeights(1e-7, 1e-8, 1e-9);
    takeIntoAccountGravity(true);
}

/** Destructor
 */
ISIRController::~ISIRController()
{

    if (pimpl->GSHCConstraintIsActive)
    {
        pimpl->innerSolver.removeConstraint(gshcFunction.getConstraint());
        pimpl->GSHCConstraintIsActive = false;
        gshcFunction.getFunction().deleteVariables();

        pimpl->var.clear();

    }

    for (int i=0; i<pimpl->createdTask.size(); ++i)
    {
        pimpl->createdTask[i]->disconnectFromController();
        delete pimpl->createdTask[i];

    }

    if (!pimpl->reducedProblem)
    {
        pimpl->innerSolver.removeConstraint(pimpl->dynamicEquation.getConstraint());
        pimpl->innerSolver.removeObjective(pimpl->minDdqObjective);
        pimpl->innerSolver.removeObjective(pimpl->minTauObjective);
        pimpl->innerSolver.removeObjective(pimpl->minFcObjective);
    }
    else
    {
        pimpl->innerSolver.removeObjective(pimpl->minTauObjective);
        pimpl->innerSolver.removeObjective(pimpl->minFcObjective);
    }

}


/**
 * \return the inner model used to construct this controller instance
 */
Model& ISIRController::getModel()
{
    return pimpl->innerModel;
}

/**
 * \return the inner solver used to construct this controller instance
 */
ISIRSolver& ISIRController::getSolver()
{
    return pimpl->innerSolver;
}


/**
 * \return \c true if variable of reduced problem (\f$ [ \torque \; \force_c ] \f$) is considered
 */
bool ISIRController::isUsingReducedProblem()
{
    return pimpl->reducedProblem;
}



void ISIRController::setVariableMinimizationWeights(double w_ddq, double w_tau, double w_fc)
{
    pimpl->minDdqObjective.getObjective().setWeight(w_ddq);
    pimpl->minTauObjective.getObjective().setWeight(w_tau);
    pimpl->minFcObjective.getObjective().setWeight(w_fc);
}

void ISIRController::takeIntoAccountGravity(bool useGrav)
{
    pimpl->dynamicEquation.getFunction().takeIntoAccountGravity(useGrav);
}



void ISIRController::addConstraint(orc::LinearConstraint& constraint) const
{
    pimpl->innerSolver.addConstraint(constraint);
}


void ISIRController::removeConstraint(orc::LinearConstraint& constraint) const
{
    pimpl->innerSolver.removeConstraint(constraint);
}

void ISIRController::addConstraint(ISIRConstraint& constraint) const
{
    constraint.connectToController(pimpl->dynamicEquation, pimpl->reducedProblem);
    pimpl->innerSolver.addConstraint(constraint.getConstraint());
}


void ISIRController::removeConstraint(ISIRConstraint& constraint) const
{
    pimpl->innerSolver.removeConstraint(constraint.getConstraint());
    constraint.disconnectFromController();
}



/** Internal implementation inside the addTask method.
 *
 * \param task The task to add in the controller
 */
void ISIRController::doAddTask(Task& task)
{
    try {
        ISIRTask& ctask = dynamic_cast<ISIRTask&>(task);
        ctask.connectToController(pimpl->innerSolver, pimpl->dynamicEquation, pimpl->reducedProblem, pimpl->useGSHC);
    }
    catch(const std::exception & e) {
        std::cerr << e.what() ;
        throw std::runtime_error("[ISIRController::doAddTask] cannot add task to controller (wrong type)");
    }
}

/** Internal implementation inside the addContactSet method.
 *
 * \param contacts The contact set to add in the controller
 * \todo this method has not been implemented!!!
 */
void ISIRController::doAddContactSet(const ContactSet& contacts)
{
    throw std::runtime_error("[ISIRController::doAddTask] not implemented");
}




ISIRTask& ISIRController::createISIRTask(const std::string& name, const Feature& feature, const Feature& featureDes) const
{
    return dynamic_cast<ISIRTask&>(createTask(name, feature, featureDes));
}

ISIRTask& ISIRController::createISIRTask(const std::string& name, const Feature& feature) const
{
    return dynamic_cast<ISIRTask&>(createTask(name, feature));
}

ISIRTask& ISIRController::createISIRContactTask(const std::string& name, const PointContactFeature& feature, double mu, double margin) const
{
    return dynamic_cast<ISIRTask&>(createContactTask(name, feature, mu, margin));
}





/** Internal implementation inside the createTask method.
 *
 * \param name The task name, a unique identifier
 * \param feature The part of the robot one wants to control (full state, frame, CoM,...)
 * \param featureDes The desired state one wants to reach, depends on the \a feature argument
 * \return The pointer to the new created task
 *
 * This method is called by the higher level methods #createISIRTask(const std::string&, const Feature&, const Feature&, int, double) const
 * and #createISIRTask(const std::string&, const Feature&, int, double) const and is the concrete implementation required by the xde Controller class.
 */
Task* ISIRController::doCreateTask(const std::string& name, const Feature& feature, const Feature& featureDes) const
{
    ISIRTask* nTask = new ISIRTask(name, pimpl->innerModel, feature, featureDes);
    pimpl->createdTask.push_back(nTask);
    return nTask;
}

/** Internal implementation inside the createTask method.
 *
 * \param name The task name, a unique identifier
 * \param feature The part of the robot one wants to control (full state, frame, CoM,...)
 * \return The pointer to the new created task
 *
 * This method is called by the higher level methods #createISIRTask(const std::string&, const Feature&, const Feature&, int, double) const
 * and #createISIRTask(const std::string&, const Feature&, int, double) const and is the concrete implementation required by the xde Controller class.
 */
Task* ISIRController::doCreateTask(const std::string& name, const Feature& feature) const
{
    ISIRTask* nTask = new ISIRTask(name, pimpl->innerModel, feature);
    pimpl->createdTask.push_back(nTask);
    return nTask;
}

/** Internal implementation inside the createContactTask method.
 *
 * \param name The task name, a unique identifier
 * \param feature The contact point feature of the robot one wants to control
 * \param mu The friction cone coefficient \f$ \mu \f$ such as \f$ \Vert \force_t \Vert < \mu \force_n \f$
 * \param margin The margin inside the friction cone
 * \return The pointer to the new created contact task
 *
 * This method is called by the higher level methods #createISIRContactTask(const std::string&, const PointContactFeature&, , double, double, int, double) const
 * and is the concrete implementation required by the xde Controller class.
 */
Task* ISIRController::doCreateContactTask(const std::string& name, const PointContactFeature& feature, double mu, double margin) const
{
    ISIRTask* nTask = new ISIRTask(name, pimpl->innerModel, feature);
    pimpl->createdTask.push_back(nTask);
    return nTask;
}




//MOST IMPORTANT FUNCTION: COMPUTE OUTPUT TORQUE
/** Compute the output of the controller.
 *
 * \param tau The torque variable, which is the output of our problem
 *
 * Here, the controller solves the optimization problem depending on the tasks and constraints, and the result is set in the variable of the problem,
 * either \f$ x = [ \ddq \; \tau \; \force_c ] \f$ or \f$ x = [ \tau \; \force_c ] \f$. The torque variable \f$ \tau \f$ is finally applied to the robot.
 */
void ISIRController::doComputeOutput(Eigen::VectorXd& tau)
{
    pimpl->updateTasksRecorder.initializeTime();
    const std::vector<Task*>& tasks = getActiveTasks();
    for(int i=0; i< tasks.size(); i++)
    {
        ISIRTask* cTask = static_cast<ISIRTask*>(tasks[i]); // addTask throws if this cast is not possible
        cTask->update();
    }
    pimpl->updateTasksRecorder.saveRelativeTime();

    pimpl->solveProblemRecorder.initializeTime();
    if(!pimpl->innerSolver.solve().info)
    {
        tau = pimpl->innerModel.getJointTorqueVariable().getValue();
    }
    else
    {
        setErrorMessage("solver error");
        setErrorFlag(OTHER | CRITICAL_ERROR);
    }
    pimpl->solveProblemRecorder.saveRelativeTime();
}

/** Write information about controller performances in a string stream.
 *
 * \param outstream the output stream where to write the performances information
 * \param addCommaAtEnd If true, add a comma at the end of the stream. If false, it means that this is the end of the json file, nothing will be added after that, no comma is added.
 *
 * See orcisir::Orocos_ISIRController::getPerformances() to know more. Here it saves:
 *
 *  - controller_update_tasks
 *  - controller_solve_problem
 */
void ISIRController::writePerformanceInStream(std::ostream& outstream, bool addCommaAtEnd) const
{
    pimpl->updateTasksRecorder.writeInStream("controller_update_tasks", outstream, true);
    pimpl->solveProblemRecorder.writeInStream("controller_solve_problem", outstream, true);
    pimpl->innerSolver.writePerformanceInStream(outstream, addCommaAtEnd);
}



/** Get information about performances through a string.
 *
 * Information are saved in a JSON way (http://www.json.org/). It returns a of dictionnary on the form:
 *
 * \code
 * {
 *    "performance_info_1": [0.01, 0.02, 0.01, ... , 0.03, 0.01],
 *    ...
 *    "performance_info_n": [0.01, 0.02, 0.01, ... , 0.03, 0.01]
 * }
 * \endcode
 *
 * where performance_info are:
 *
 *  - controller_update_tasks
 *  - controller_solve_problem
 *  - solver_prepare
 *  - solver_solve
 *
 * See orcisir::ISIRController::writePerformanceInStream(std::ostream&, bool) and orcisir::ISIRSolver::writePerformanceInStream(std::ostream&, bool).
 */
std::string ISIRController::getPerformances() const
{
    std::ostringstream osstream;
    // Write it in a json style
    osstream <<"{\n";
    writePerformanceInStream(osstream, false);
    osstream <<"}";
    return osstream.str();
}

void ISIRController::setActiveTaskVector()
{
    pimpl->activeTask.clear();
    int totalTaskDim = 0;
    for (int i=0; i<pimpl->createdTask.size(); ++i)
    {
        if (pimpl->createdTask[i]->isActiveAsObjective())
        {
            if(pimpl->createdTask[i]->getTaskType()==orcisir::ISIRTask::FORCETASK||
                pimpl->createdTask[i]->getTaskType()==orcisir::ISIRTask::ACCELERATIONTASK&&
                !pimpl->createdTask[i]->isPointContactTask())
            {
                pimpl->activeTask.push_back(pimpl->createdTask[i]);
                totalTaskDim += pimpl->createdTask[i]->getTaskDimension();
                pimpl->createdTask[i]->setWeight(1);
            }
        }
    }
    pimpl->nbActiveTask = pimpl->activeTask.size();
    pimpl->totalActiveTaskDim = totalTaskDim;

//    //compute augmented jacobian
//    int nDof = pimpl->innerModel.nbDofs();
//    MatrixXd jacobian(totalTaskDim, nDof);
//    int rowindex = 0;
//    int taskdim;


//    for (int i=0; i<pimpl->nbActiveTask; ++i)
//    {
//        taskdim = pimpl->activeTask[i]->getDimension();
//        jacobian.block(rowindex,0, taskdim, nDof)=pimpl->activeTask[i]->getJacobian();
//        rowindex += taskdim;
//    }
//    pimpl->augmentedJacobian = jacobian;
    doUpdateAugmentedJacobian();
    getTaskVariables();
    initPriorityMatrix();

}

void ISIRController::doUpdateAugmentedJacobian()
{
     int nDof = pimpl->innerModel.nbDofs();
     MatrixXd jacobian=MatrixXd::Zero(pimpl->totalActiveTaskDim, nDof);
     int rowindex = 0;
     int taskdim;
//     int index = pimpl->innerModel.hasFixedRoot() ? 0 : 6;

     for (int i=0; i<pimpl->nbActiveTask; ++i)
     {
         taskdim = pimpl->activeTask[i]->getDimension();
         jacobian.block(rowindex,0, taskdim, nDof)=pimpl->activeTask[i]->getJacobian();        
         rowindex += taskdim;
     }
     pimpl->augmentedJacobian = jacobian;
}

void ISIRController::doUpdateProjector()
{
     doUpdateAugmentedJacobian();
     const int nDof = pimpl->innerModel.nbDofs();
     MatrixXd proj(nDof,nDof);
     MatrixXd taskiProj(nDof,nDof);
     for (int i=0; i<pimpl->nbActiveTask; ++i)
     {
         computeProjector(pimpl->activeTask[i]->getPriority(), pimpl->augmentedJacobian, proj);
         pimpl->activeTask[i]->setProjector(proj);
         computeTaskiProjector(pimpl->activeTask[i]->getJacobian(), taskiProj);
         pimpl->activeTask[i]->setTaskiProjector(taskiProj);
     }
}
std::vector< ISIRTask* >& ISIRController::getActiveTask()
{
    return pimpl->activeTask;
}

int ISIRController::getNbActiveTask() const
{
    return pimpl->nbActiveTask;
}

int ISIRController::getTotalActiveTaskDimensions() const
{
    return pimpl->totalActiveTaskDim;
}

void ISIRController::initPriorityMatrix()
{
    int dim;
    int dim_alpha = 0;
    for (int i=0; i<pimpl->activeTask.size(); ++i)
    {
        dim = pimpl->activeTask[i]->getTaskDimension();
        pimpl->activeTask[i]->setIndexBegin(dim_alpha);
        pimpl->activeTask[i]->setIndexEnd(dim_alpha + dim);
        dim_alpha += dim;
    }
//    MatrixXd m_alpha = MatrixXd::Zero(dim_alpha,dim_alpha);

}

std::pair<VectorXd, MatrixXd> ISIRController::sortRows(const MatrixXd &C, const MatrixXd &J)
{
    int totalTaskDim = J.rows();
    int nDof = J.cols();
    MatrixXd Cii_J = MatrixXd::Zero(totalTaskDim, 1+nDof);

    Cii_J.col(0) = C.diagonal();
    Cii_J.block(0,1,totalTaskDim,nDof) = J;

    int rdim = Cii_J.rows();
    VectorXd tmp(1, 1+nDof);
    for (int rmin=0; rmin<rdim-1;++rmin)
    {
        for(int i=rdim-1; i>rmin; --i)
        {
            if (Cii_J(i,0)>Cii_J(i-1,0))
            {
                tmp = Cii_J.row(i-1);
                Cii_J.row(i-1) = Cii_J.row(i);
                Cii_J.row(i) = tmp;
            }
        }
    }

    return std::pair<VectorXd, MatrixXd>(Cii_J.col(0),Cii_J.block(0,1,totalTaskDim,nDof));
}

void ISIRController::computeProjector(const MatrixXd &C, const MatrixXd &J, MatrixXd& projector)
{
    int totalTaskDim = J.rows();
    const int nDof = pimpl->innerModel.nbDofs();
    VectorXd Cs(nDof);
    MatrixXd Js(totalTaskDim, nDof);

    std::pair<VectorXd, MatrixXd> sortedMatrix = sortRows(C,J);
    Cs = sortedMatrix.first;
    Js = sortedMatrix.second;
    orcisir::OrthonormalFamily onfamily(Js, 1e-9);
    onfamily.computeOrthonormalFamily();
    MatrixXd onb_Js = onfamily.getOnf();
    VectorXi origin = onfamily.getOrigin();
//    std::cout<<"origin"<<std::endl;
//    std::cout<<origin<<std::endl;
    int k = onfamily.getIndex();
    VectorXd Chat(k);

    for (int j=0; j<k; ++j)
    {
        Chat(j)=Cs(origin(j));
    }


    MatrixXd alpha = MatrixXd(Chat.asDiagonal());
//    std::cout<<"Cs"<<std::endl;
//    std::cout<<Cs<<std::endl;
//    std::cout<<"Chat"<<std::endl;
//    std::cout<<Chat<<std::endl;
//    std::cout<<"onb_js"<<std::endl;
//    std::cout<<onb_Js<<std::endl;
//    std::cout<<"alpha"<<std::endl;
//    std::cout<<alpha<<std::endl;
    projector = MatrixXd::Identity(nDof,nDof) - onb_Js.transpose()*alpha*onb_Js;
}

void ISIRController::computeTaskiProjector(const MatrixXd &J, MatrixXd& projector)
{

    orcisir::OrthonormalFamily onfamily(J, 1e-9);
    onfamily.computeOrthonormalFamily();
    MatrixXd onb_Js = onfamily.getOnf();
    const int nDof = pimpl->innerModel.nbDofs();

    projector = MatrixXd::Identity(nDof,nDof) - onb_Js.transpose()*onb_Js;
}

CompositeVariable& ISIRController::getTaskVariables()
{
    pimpl->var.clear();
    pimpl->var.add(pimpl->innerModel.getAccelerationVariable());
    for (int i=0; i<pimpl->nbActiveTask; ++i)
    {
        pimpl->var.add(pimpl->activeTask[i]->getTaskAccelerationVariable());
    }
    return pimpl->var;
}

void ISIRController::setTaskProjectors(MatrixXd& param)
{
//    initPriorityMatrix(param);

    VectorXd vec_ij;
    int dim_j;
    MatrixXd m_priority(pimpl->totalActiveTaskDim,pimpl->totalActiveTaskDim);
    m_priority.setZero();

    int nt = pimpl->nbActiveTask;
    for (int i=0; i<nt; ++i)
    {
        for (int j=0; j<nt; ++j)
        {
            dim_j = pimpl->activeTask[j]->getTaskDimension();
            vec_ij = VectorXd::Zero(dim_j);
            vec_ij = vec_ij.setConstant(param(i,j));
            m_priority.block(pimpl->activeTask[j]->getIndexBegin(), pimpl->activeTask[j]->getIndexBegin(),dim_j,dim_j) = MatrixXd(vec_ij.asDiagonal());
        }

        pimpl->activeTask[i]->setPriority(m_priority);

    }

    const int nDof = pimpl->innerModel.nbDofs();
    MatrixXd proj(nDof,nDof);
    MatrixXd taskiProj(nDof,nDof);

    for (int i=0; i<pimpl->nbActiveTask; ++i)
    {
        computeProjector(pimpl->activeTask[i]->getPriority(), pimpl->augmentedJacobian, proj);
        pimpl->activeTask[i]->setProjector(proj);
        computeTaskiProjector(pimpl->activeTask[i]->getJacobian(), taskiProj);
        pimpl->activeTask[i]->setTaskiProjector(taskiProj);
    }

}

void ISIRController::setGSHCConstraint()
{
//    if (pimpl->GSHCConstraintIsActive)
//        pimpl->innerSolver.removeConstraint(pimpl->gshcFunction.getConstraint());

//    gshcFunction = new GSHCFunction(pimpl->innerModel, pimpl->activeTask, pimpl->var);//.getFunction().setActiveTaskAndVariables(pimpl->activeTask, pimpl->var);
    gshcFunction = new orcisir::GSHCFunction(pimpl->innerModel, pimpl->activeTask, pimpl->var);
    addConstraint(gshcFunction.getConstraint());
    pimpl->GSHCConstraintIsActive = true;
}

} // namespace orcisir

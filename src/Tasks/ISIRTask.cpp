/**
 * \file ISIRTask.cpp
 * \author Joseph Salini
 *
 * \brief Implement \b task class for ISIR controller. It inherits from the task class defined in the xde framework.
 *
 * File history
 * - 14/06/13: Mingxing Liu - Adaptation to generalized smooth hierarchical controller (GSHC).
 * - 15/01/26: Mingxing Liu - Allow the control of non-sliding contact forces by adding forceControlObjective
 */

#include "orcisir/Tasks/ISIRTask.h"

// ORC INCLUDES
#include "orc/control/Model.h"
#include "orc/optim/FunctionHelpers.h"
#include "orc/optim/LinearizedCoulombFunction.h"
#include "orc/optim/WeightedSquareDistanceFunction.h"

// ORCISIR INCLUDES
#include "orcisir/ISIRDebug.h"


namespace orcisir
{


class VariableChiFunction : public orc::LinearFunction
{
public:
    VariableChiFunction(orc::Variable& x, int dimension)
    : NamedInstance("Variable Chi Linear Function")
    , orc::AbilitySet(orc::PARTIAL_X)
    , CoupledInputOutputSize(false)
    , LinearFunction(x, dimension)
    {

    }

    void doUpdateInputSizeBegin() {};

    void doUpdateInputSizeEnd() {};
};


struct ISIRTask::Pimpl
{
    const std::string&          name;
    const Model&                innerModel;
    ISIRSolver*                 solver;
    const ISIRDynamicFunction*  dynamicEquation;
    bool                        useReducedProblem;
    bool                        useGSHC;
    orc::BaseVariable           fcVar;
    orc::BaseVariable           ddqi;
    orc::CompositeVariable      fcVarddqi;

    Eigen::Wrenchd              fRef; // reference force for force control
    bool                  doForceControl;
    double                weight;
    MatrixXd              alpha;
    MatrixXd              projector;
    MatrixXd              taskiProjector;
    const Feature&        feature;
    int                   indexBegin;
    int                   indexEnd;

    bool taskHasBeenInitialized;

    LessThanZeroConstraintPtr<LinearizedCoulombFunction>    frictionConstraint; // if contact task

    bool contactForceConstraintHasBeenSavedInSolver;
    bool contactPointHasBeenSavedInModel;
    bool frictionConstraintIsRegisteredInConstraint;
    EqualZeroConstraintPtr<LinearFunction>  ContactForceConstraint;

    bool isRegisteredAsObjective;
    bool isRegisteredAsConstraint;
    bool isPointContactTask;

    TYPETASK                                innerTaskType;

    LinearFunction*                         innerObjectiveFunction;
    Objective<SquaredLinearFunction>*       innerTaskAsObjective;
    EqualZeroConstraintPtr<LinearFunction>  innerTaskAsConstraint;

    LinearFunction*                         regulationObjectiveFunction; //objective function for GSHC
    Objective<SquaredLinearFunction>*       regulationObjective;
    ObjectivePtr<WeightedSquareDistanceFunction> forceControlObjective; //regulate contact force
    bool forceControlObjectiveHasBeenSavedInSolver;

    Pimpl(const std::string& name, const Model& m, const Feature& s)
        : name(name)
        , innerModel(m)
        , solver(0x0)
        , dynamicEquation(0x0)
        , useReducedProblem(false)
        , useGSHC(false)
        , fcVar(name+".var", s.getDimension())
        , ddqi(name+"_ddqi.var", m.nbDofs())
        , fcVarddqi(name+"fc_ddqi.var",fcVar,ddqi)
        , fRef(Wrenchd::Zero())
        , doForceControl(false)
        , weight(1.)
        , alpha(MatrixXd::Zero(s.getDimension(),s.getDimension()))
        , projector(MatrixXd::Identity(m.nbDofs(),m.nbDofs()))
        , taskiProjector(MatrixXd::Identity(m.nbDofs(),m.nbDofs()))
        , feature(s)
        , taskHasBeenInitialized(false)
        , contactForceConstraintHasBeenSavedInSolver(false)
        , contactPointHasBeenSavedInModel(false)
        , frictionConstraintIsRegisteredInConstraint(false)
        , isRegisteredAsObjective(false)
        , isRegisteredAsConstraint(false)
        , isPointContactTask(false)
        , innerTaskType(UNKNOWNTASK)
        , innerObjectiveFunction(NULL)
        , innerTaskAsObjective(NULL)
        , regulationObjectiveFunction(NULL)
        , regulationObjective(NULL)
        , forceControlObjectiveHasBeenSavedInSolver(false)
    {
        innerTaskAsConstraint.set(NULL);

        if(fcVar.getSize() == 3)
        {
//            std::cout<<"CAN BE A CONTACT POINT!!! register friction and contact constraints\n";
//            registerFrictionConstraint = true;
            frictionConstraint.set(  new LinearizedCoulombFunction(fcVar, 1., 6, 0.) );
            ContactForceConstraint.set( new LinearFunction( fcVar, Eigen::MatrixXd::Identity(3,3), VectorXd::Zero(3) ) );
            forceControlObjective.set( new WeightedSquareDistanceFunction(fcVar, 1., VectorXd::Zero(feature.getDimension())), weight);

        }
        else
        {
//            registerFrictionConstraint = false;
            frictionConstraint.set(NULL);
            ContactForceConstraint.set(NULL);
            forceControlObjective.set(NULL);
        }
    }

    ~Pimpl()
    {
        if (innerTaskAsObjective)
        {
            delete &innerTaskAsObjective->getFunction();
            delete innerTaskAsObjective;
        }
        if (regulationObjective)
        {
            std::cout<<"----"<<std::endl;

            delete &regulationObjective->getFunction();
            delete regulationObjective;
        }

    }

    void setAsAccelerationTask()
    {
        int featn = feature.getDimension();
        if (useReducedProblem)
        {
            innerObjectiveFunction = new VariableChiFunction(dynamicEquation->getActionVariable(), featn);
        }
        else
        {
            if (useGSHC&&!isPointContactTask)
            {
                innerObjectiveFunction = new LinearFunction(ddqi, Eigen::MatrixXd::Zero(featn, innerModel.nbDofs()), Eigen::VectorXd::Zero(featn));
                regulationObjectiveFunction = new LinearFunction(ddqi, Eigen::MatrixXd::Identity(innerModel.nbDofs(), innerModel.nbDofs()),Eigen::VectorXd::Zero(innerModel.nbDofs()));
            }
            else
                innerObjectiveFunction = new LinearFunction(innerModel.getAccelerationVariable(), Eigen::MatrixXd::Zero(featn, innerModel.nbDofs()), Eigen::VectorXd::Zero(featn));
        }
        connectFunctionnWithObjectiveAndConstraint();
    }

    void setAsTorqueTask()
    {
        int featn = feature.getDimension();
        innerObjectiveFunction = new LinearFunction(innerModel.getJointTorqueVariable(), Eigen::MatrixXd::Zero(featn, innerModel.nbInternalDofs()), Eigen::VectorXd::Zero(featn));
        connectFunctionnWithObjectiveAndConstraint();
    }

    void setAsForceTask()
    {
        int featn = feature.getDimension();
        if (useGSHC)
        {
            innerObjectiveFunction = new LinearFunction(fcVarddqi, Eigen::MatrixXd::Zero(innerModel.nbDofs()+featn, innerModel.nbDofs()+featn), Eigen::VectorXd::Zero(innerModel.nbDofs()+featn));
//            regulationObjectiveFunction = new LinearFunction(fcVarddqi, Eigen::MatrixXd::Identity(innerModel.nbDofs()+featn, innerModel.nbDofs()+featn),Eigen::VectorXd::Zero(innerModel.nbDofs()+featn));
//            innerObjectiveFunction = new LinearFunction(ddqi, Eigen::MatrixXd::Zero(featn, innerModel.nbDofs()), Eigen::VectorXd::Zero(featn));
            regulationObjectiveFunction = new LinearFunction(ddqi, Eigen::MatrixXd::Identity(innerModel.nbDofs(), innerModel.nbDofs()),Eigen::VectorXd::Zero(innerModel.nbDofs()));
        }
        else
            innerObjectiveFunction = new LinearFunction(fcVar, Eigen::MatrixXd::Identity(featn, featn), Eigen::VectorXd::Zero(featn));
        connectFunctionnWithObjectiveAndConstraint();
    }

    void connectFunctionnWithObjectiveAndConstraint()
    {
        innerTaskAsObjective = new Objective<SquaredLinearFunction>(new SquaredLinearFunction(innerObjectiveFunction), weight); // Here, it will manage the SquaredLinearFunction build on a pointer of the function.
        innerTaskAsConstraint.set(innerObjectiveFunction);      // As as ConstraintPtr, it will manage the new created function innerObjectiveFunction

        if (useGSHC)
        {
            if (innerTaskType==FORCETASK||innerTaskType==ACCELERATIONTASK&&!isPointContactTask)
            {
                regulationObjective = new Objective<SquaredLinearFunction>(new SquaredLinearFunction(regulationObjectiveFunction), weight*0.01);
//                regulationObjective = new Objective<SquaredLinearFunction>(new SquaredLinearFunction(regulationObjectiveFunction), 1.0);
            }

        }
    }
};


/** Initialize a new ISIR Task.
 *
 * \param name The name of the task
 * \param model The xde model on which we will update the dynamic parameters
 * \param feature The task feature, meaning what we want to control
 * \param featureDes The desired task feature, meaning the goal we want to reach with the \a feature
 */
ISIRTask::ISIRTask(const std::string& taskName, const Model& innerModel, const Feature& feature, const Feature& featureDes)
    : Task(taskName, innerModel, feature, featureDes)
    , pimpl(new Pimpl(taskName, innerModel, feature))
{

}

/** Initialize a new ISIR Task.
 *
 * \param name The name of the task
 * \param model The xde model on which we will update the dynamic parameters
 * \param feature The task feature, meaning what we want to control
 */
ISIRTask::ISIRTask(const std::string& taskName, const Model& innerModel, const Feature& feature)
    : Task(taskName, innerModel, feature)
    , pimpl(new Pimpl(taskName, innerModel, feature))
{

}


ISIRTask::~ISIRTask()
{
//    if (pimpl->innerTaskType==ACCELERATIONTASK&&pimpl->useGSHC&&!isPointContactTask())
//    {
//        delete &getTaskAccelerationVariable();
//    }
}


void ISIRTask::connectToController(ISIRSolver& solver, const ISIRDynamicFunction& dynamicEquation, bool useReducedProblem, bool useGSHC)
{
    pimpl->solver            = &solver;
    pimpl->dynamicEquation   = &dynamicEquation;
    pimpl->useReducedProblem =  useReducedProblem;
    pimpl->useGSHC           =  useGSHC;

    switch(pimpl->innerTaskType)
    {

        case(ACCELERATIONTASK):
        {
            pimpl->isPointContactTask = isPointContactTask();
            pimpl->setAsAccelerationTask();
            break;
        }
        case(TORQUETASK):
        {
            pimpl->setAsTorqueTask();
            break;
        }
        case(FORCETASK):
        {
            pimpl->setAsForceTask();
            break;
        }
        case(UNKNOWNTASK):
        {
            std::string errmsg = std::string("[ISIRTask::connectToController]: The task type of '") + getName() + std::string("' has not been set during creation.\nCall prior that 'initAsAccelerationTask', 'initAsTorqueTask' or 'initAsForceTask'\n"); //
            throw std::runtime_error(std::string(errmsg));
            break;
        }
        default:
        {
            throw std::runtime_error(std::string("[ISIRTask::connectToController]: Unhandle case of TYPETASK for task ")+getName() );
            break;
        }
    }
}


void ISIRTask::disconnectFromController()
{
    if (pimpl->isRegisteredAsObjective)
    {
        pimpl->solver->removeObjective(*pimpl->innerTaskAsObjective);
        if (pimpl->useGSHC)
        {
            if (pimpl->innerTaskType==FORCETASK||pimpl->innerTaskType==ACCELERATIONTASK&&!isPointContactTask())
            {
                pimpl->solver->removeObjective(*pimpl->regulationObjective);
            }
        }

    }
    if (pimpl->isRegisteredAsConstraint)
    {
        pimpl->solver->removeConstraint(pimpl->innerTaskAsConstraint);
    }

    if (pimpl->frictionConstraintIsRegisteredInConstraint)
    {
        pimpl->solver->removeConstraint(pimpl->frictionConstraint);
    }
    if (pimpl->contactForceConstraintHasBeenSavedInSolver)
    {
        pimpl->solver->removeConstraint(pimpl->ContactForceConstraint);
    }

}



void ISIRTask::initAsAccelerationTask()
{
    pimpl->innerTaskType = ACCELERATIONTASK;
}


void ISIRTask::initAsTorqueTask()
{
    pimpl->innerTaskType = TORQUETASK;
}


void ISIRTask::initAsForceTask()
{
    pimpl->innerTaskType = FORCETASK;
}




ISIRTask::TYPETASK ISIRTask::getTaskType() const
{
    return pimpl->innerTaskType;
}


const Eigen::VectorXd& ISIRTask::getComputedForce() const
{
    return pimpl->fcVar.getValue();
}


void ISIRTask::doGetOutput(Eigen::VectorXd& output) const
{
    if (pimpl->useGSHC)
    {
        output = pimpl->ddqi.getValue();
    }
    else
        output = Eigen::VectorXd::Zero(getDimension());
}



void ISIRTask::addContactPointInModel()
{
    //THIS SHOULD BE DONE ONLY ONCE!!!
    if ( ! pimpl->contactPointHasBeenSavedInModel )
    {
        pimpl->innerModel.getModelContacts().addContactPoint(pimpl->fcVar, getFeature());
        pimpl->contactPointHasBeenSavedInModel = true;
    }

    if ( pimpl->contactForceConstraintHasBeenSavedInSolver )
    {
        pimpl->solver->removeConstraint(pimpl->ContactForceConstraint);
        pimpl->contactForceConstraintHasBeenSavedInSolver = false;
    }
}

void ISIRTask::removeContactPointInModel()
{
    //    if ( pimpl->contactPointHasBeenSavedInModel )
//    {
//        pimpl->model.getModelContacts().removeContactPoint(pimpl->fcVar);
//        pimpl->contactPointHasBeenSavedInModel = false;
//    }
    if ( ! pimpl->contactForceConstraintHasBeenSavedInSolver )
    {
        pimpl->solver->addConstraint(pimpl->ContactForceConstraint);
        pimpl->contactForceConstraintHasBeenSavedInSolver = true;
    }
}



/** Do task activation when it is a contact task.
 *
 * When this function is called, it adds a contact point in the model of contact contained in the xde Model instance,
 * and it adds in the solver an inequality constraint that represents the limitation of the contact force that must remain inside the cone of friction.
 */
void ISIRTask::doActivateContactMode()
{
    checkIfConnectedToController();

    addContactPointInModel();

    // add friction cone in constraint
    pimpl->solver->addConstraint(pimpl->frictionConstraint);
    pimpl->frictionConstraintIsRegisteredInConstraint = true;
    if (pimpl->doForceControl&&(!pimpl->forceControlObjectiveHasBeenSavedInSolver) )
    {
        pimpl->solver->addObjective(pimpl->forceControlObjective.getObjective());
        pimpl->forceControlObjectiveHasBeenSavedInSolver = true;
    }

}


/** Do task deactivation when it is a contact task.
 *
 * When this function is called, it removes the contact point in the model of contact,
 * and it removes from the solver the friction cone inequality constraint.
 */
void ISIRTask::doDeactivateContactMode()
{
    checkIfConnectedToController();

    removeContactPointInModel();

    // remove friction cone from constraint set
    pimpl->solver->removeConstraint(pimpl->frictionConstraint);
    pimpl->frictionConstraintIsRegisteredInConstraint = false;
    if (pimpl->doForceControl&&pimpl->forceControlObjectiveHasBeenSavedInSolver)
    {
        pimpl->solver->removeObjective(pimpl->forceControlObjective.getObjective());
        pimpl->forceControlObjectiveHasBeenSavedInSolver = false;
    }

}


/** For contact task, do the setting of the coefficient of friction.
 *
 * The cone of friction constraint is modified to represent a cone with this new coefficient of friction.
 */
void ISIRTask::doSetFrictionCoeff()
{
    pimpl->frictionConstraint.getFunction().setFrictionCoeff(getFrictionCoeff());
}

/** For contact task, do the setting of the friction margin.
 *
 * The cone of friction constraint is modified to represent a friction cone with this new margin.
 */
void ISIRTask::doSetMargin()
{
    pimpl->frictionConstraint.getFunction().setMargin(getMargin());
}









/** Do activation of task as an objective.
 *
 * It means that the task is not fully completed and a little error may occur.
 */
void ISIRTask::doActivateAsObjective()
{
    checkIfConnectedToController();
    pimpl->solver->addObjective(*pimpl->innerTaskAsObjective);
    if (pimpl->useGSHC)
    {
        if(pimpl->innerTaskType==FORCETASK||pimpl->innerTaskType==ACCELERATIONTASK&&!isPointContactTask())
        {
            pimpl->solver->addObjective(*pimpl->regulationObjective);
        }
    }

    pimpl->isRegisteredAsObjective = true;

    if (pimpl->innerTaskType == FORCETASK)
    {
        addContactPointInModel();
    }
}

/** Do deactivation of task as an objective.
 *
 * objective is no more considered.
 */
void ISIRTask::doDeactivateAsObjective()
{
    checkIfConnectedToController();
    pimpl->solver->removeObjective(*pimpl->innerTaskAsObjective);
    if (pimpl->useGSHC)
    {
        if (pimpl->innerTaskType==FORCETASK||pimpl->innerTaskType==ACCELERATIONTASK&&!isPointContactTask())
        {
            pimpl->solver->removeObjective(*pimpl->regulationObjective);
        }
    }
    pimpl->isRegisteredAsObjective = false;

    if (pimpl->innerTaskType == FORCETASK)
    {
        removeContactPointInModel();
    }
}

/** Do activation of task as a constraint.
 *
 * It means that the task should be full completed and no error may occur.
 * Be aware that stong constraints may lead to system instability (very "sharp" solution that requires lot of energy).
 */
void ISIRTask::doActivateAsConstraint()
{
    checkIfConnectedToController();
    pimpl->solver->addConstraint(pimpl->innerTaskAsConstraint);
    pimpl->isRegisteredAsConstraint = true;

    if (pimpl->innerTaskType == FORCETASK)
    {
        addContactPointInModel();
    }
}

/** Do deactivation of task as a constraint.
 *
 * objective is no more considered.
 */
void ISIRTask::doDeactivateAsConstraint()
{
    checkIfConnectedToController();
    pimpl->solver->removeConstraint(pimpl->innerTaskAsConstraint);
    pimpl->isRegisteredAsConstraint = false;

    if (pimpl->innerTaskType == FORCETASK)
    {
        removeContactPointInModel();
    }
}



/** Do set the weight of the task.
 *
 * The weight in the objective function is modified.
 *
 * \todo the \b getWeight() method of the task returns a matrix, but the \b setWeight() method of the objective requires a double value.
 * I don't really know how to cope with this problem. For now the first value of the matrix is used.
 */
void ISIRTask::doSetWeight()
{
    pimpl->weight = getWeight()[0]; //TODO: BUG HERE!!!
    if (pimpl->innerTaskAsObjective)
    {
        pimpl->innerTaskAsObjective->setWeight(pimpl->weight);
    }

}


//--------------------------------------------------------------------------------------------------------------------//
void ISIRTask::update()
{
    switch(pimpl->innerTaskType)
    {

        case(ACCELERATIONTASK):
        {
            doUpdateAccelerationTask();
            break;
        }
        case(TORQUETASK):
        {
            doUpdateTorqueTask();
            break;
        }
        case(FORCETASK):
        {
            doUpdateForceTask();
            break;
        }
        case(UNKNOWNTASK):
        {
            throw std::runtime_error(std::string("[ISIRTask::update]: The task type has not been set during creation."));
            break;
        }
        default:
        {
            throw std::runtime_error(std::string("[ISIRTask::update]: Unhandle case of TYPETASK."));
            break;
        }
    }
}




/** Update linear function of the task for the full formalism.
 *
 * It computes a desired acceleration \f$ \vec{a}^{des} = - \left( \vec{a}_{ref} + K_p (\vec{p}^{des} - vec{p}) +  K_d (\vec{v}^{des} - vec{v}) \right) \f$ .
 * Then The linear function is set as follows:
 *
 * - if it use the reduced problem:
 *
 * \f{align*}{
 *       \A &= J_{task}  .  \left(  \M^{-1} \J_{\tav}\tp  \right)
 *     & \b &= \vec{a}^{des} - \left(  J_{task} \M^{-1} ( \g - \n)  \right)
 * \f}
 *
 * see \ref sec_transform_formalism for more information.
 *
 * - else:
 *
 * \f{align*}{
 *       \A &= J_{task}
 *     & \b &= \vec{a}^{des}
 * \f}
 */
void ISIRTask::doUpdateAccelerationTask()
{
    const MatrixXd& J  = getJacobian();
    const MatrixXd& Kp = getStiffness();
    const MatrixXd& Kd = getDamping();

    const VectorXd  accDes = - ( getErrorDdot() + Kp * getError() + Kd * getErrorDot() );

    if (pimpl->useReducedProblem)
    {
        const Eigen::MatrixXd E2 =        - J * pimpl->dynamicEquation->getInertiaMatrixInverseJchiT();
        const Eigen::VectorXd f2 = accDes + J * pimpl->dynamicEquation->getInertiaMatrixInverseLinNonLinGrav();

        pimpl->innerObjectiveFunction->changeA(E2);
        pimpl->innerObjectiveFunction->changeb(f2);
    }
    else
    {
        pimpl->innerObjectiveFunction->changeA(J);
        pimpl->innerObjectiveFunction->changeb(accDes);
        if (pimpl->useGSHC&&!isPointContactTask())
        {
            pimpl->regulationObjectiveFunction->changeA(getTaskiProjector());
        }
    }

    if (isPointContactTask()&&pimpl->doForceControl)
    {
        pimpl->forceControlObjective.getFunction().changeReference(pimpl->fRef.getForce());
    }
}




void ISIRTask::doUpdateTorqueTask()
{
    const MatrixXd& J    =   getJacobian();
    const VectorXd  eff  = - getEffort();

    pimpl->innerObjectiveFunction->changeA(J);
    pimpl->innerObjectiveFunction->changeb(eff);
}


void ISIRTask::doUpdateForceTask()
{
    if (pimpl->useGSHC)
    {
        const MatrixXd& J  = getJacobian();
        int featn = pimpl->feature.getDimension();
        int nbDofs = pimpl->innerModel.nbDofs();
        MatrixXd A = MatrixXd::Zero(nbDofs+featn, nbDofs+featn);
        A.block(0,0,nbDofs,featn)=J.transpose();
        A.block(0,featn,nbDofs,nbDofs) = -pimpl->innerModel.getInertiaMatrix()*getProjector();
        A.block(nbDofs,featn,featn,nbDofs) = J;
        VectorXd b = VectorXd::Zero(nbDofs+featn);
        VectorXd  eff = - getEffort();
        const VectorXd  accDes = J*pimpl->innerModel.getInertiaMatrixInverse()*J.transpose()*eff;
        b.tail(featn) = accDes;
        pimpl->innerObjectiveFunction->changeA(A);
        pimpl->innerObjectiveFunction->changeb(b);

//        const MatrixXd& J  = getJacobian();
//        VectorXd  eff = - getEffort();
//        const VectorXd  accDes = J*pimpl->innerModel.getInertiaMatrixInverse()*J.transpose()*eff;
//        pimpl->innerObjectiveFunction->changeA(J);
//        pimpl->innerObjectiveFunction->changeb(accDes);

        pimpl->regulationObjectiveFunction->changeA(getTaskiProjector());

    }
    else
    {
        //innerObjectiveFunction->changeA(); //already set in initForceTask

        const VectorXd  eff  = - getEffort();

        pimpl->innerObjectiveFunction->changeb(eff);
    }
}




void ISIRTask::checkIfConnectedToController() const
{
    if (!pimpl->solver)
    {
        std::string errmsg = std::string("[ISIRTask::doActivateAsObjective]: task '") + getName() + std::string("' not connected to any solver; Call prior that 'ISIRController::addTask' to connect to the solver inside the controller.\n"); //
        throw std::runtime_error(std::string(errmsg));
    }
}


const MatrixXd ISIRTask::getPriority() const
{
  return pimpl->alpha;
}

void ISIRTask::setPriority(Eigen::MatrixXd alpha)
{
//    std::cout<<alpha<<"\n";
    pimpl->alpha = alpha;

}

int ISIRTask::getTaskDimension() const
{
    return pimpl->feature.getDimension();
}

const std::string& ISIRTask::getTaskName() const
{
    return pimpl->name;
}

void ISIRTask::setIndexBegin(int index)
{
    pimpl->indexBegin = index;
}

int ISIRTask::getIndexBegin() const
{
    return pimpl->indexBegin;
}

void ISIRTask::setIndexEnd(int index)
{
    pimpl->indexEnd = index;
}

int ISIRTask::getIndexEnd() const
{
    return pimpl->indexEnd;
}

void ISIRTask::setProjector(MatrixXd proj)
{
    pimpl->projector = proj;
}

const MatrixXd& ISIRTask::getProjector() const
{
    return pimpl->projector;
}

void ISIRTask::setTaskiProjector(MatrixXd proj)
{
    pimpl->taskiProjector = proj;
}

const MatrixXd& ISIRTask::getTaskiProjector() const
{
    return pimpl->taskiProjector;
}

LinearFunction* ISIRTask::getInnerObjectiveFunction() const
{
    return pimpl->innerObjectiveFunction;
}

Variable& ISIRTask::getTaskAccelerationVariable() const
{
//    LinearFunction* f = getInnerObjectiveFunction();
//    return f->getVariable();
    return pimpl->ddqi;
}

void ISIRTask::doContactForceControl()
{
    pimpl->doForceControl = true;
}

void ISIRTask::setReferenceForce(const Eigen::Wrenchd& fRef)
{
    pimpl->fRef = fRef;
}

} // end namespace orcisir



/**
 * \file ISIRTask.h
 * \author Joseph Salini, Mingxing Liu
 *
 * \brief Define \b task class for ISIR controller. It inherits from the task class defined in the xde framework.
 *
 * The class defined here is an abstract classes. It is overriden to fit full problem or reduced problem.
 *
 * \see ISIRFullTask.h
 * \see ISIRReducedTask.h
 *
 * File history
 * - 14/01/23: Mingxing Liu - Adaptation to generalized smooth hierarchical controller (GSHC).
 */

#ifndef __ISIRTASK_H__
#define __ISIRTASK_H__

// ORC INCLUDES
#include "orc/optim/FunctionHelpers.h"
#include "orc/optim/SquaredLinearFunction.h"
#include "orc/optim/WeightedSquareDistanceFunction.h"


#include "orc/control/Task.h"
#include "orc/control/Feature.h"
#include "orc/control/ControlFrame.h"

#include "orcisir/Solvers/ISIRSolver.h"
#include "orcisir/Constraints/ISIRConstraint.h"


using namespace orc;

namespace orcisir
{

/** \addtogroup task
 * \{
 */

/** \brief A generic abstract task for the ISIR controller.
 *
 * The main difference with the Task class defined in the xde framework is the addition of a level parameter. Hence, a hierarchical set of tasks can be solved.
 *
 * \note This level information may have been added to the controller or the solver, not directly added to the task, and the Task class of the xde framework may have been used instead of this new class.
 *       But I think the level is the same concept as the weight (an importance), so I add it direcly in the task class, like the weight.
 *       Furthermore, writting this class helps me to better understand the xde framework.
 *
 * Concrete class are orcisir::ISIRFullTask and orcisir::ISIRReducedTask .
 */
class ISIRTask: public Task
{
public:

    enum TYPETASK { UNKNOWNTASK, ACCELERATIONTASK, TORQUETASK, FORCETASK };


    ISIRTask(const std::string& taskName, const Model& innerModel, const Feature& feature, const Feature& featureDes);
    ISIRTask(const std::string& taskName, const Model& innerModel, const Feature& feature);
    virtual ~ISIRTask();

    void initAsAccelerationTask();
    void initAsTorqueTask();
    void initAsForceTask();

    TYPETASK getTaskType() const;

    const Eigen::VectorXd& getComputedForce() const;

    const Eigen::MatrixXd getPriority() const;
    void   setPriority(Eigen::MatrixXd alpha);
    int getTaskDimension() const;
    const std::string& getTaskName() const;
    void setIndexBegin(int index);
    int getIndexBegin() const;
    void setIndexEnd(int index);
    int getIndexEnd() const;
    void setProjector(Eigen::MatrixXd proj);
    void setTaskiProjector(Eigen::MatrixXd proj);
    const Eigen::MatrixXd& getProjector() const;
    const Eigen::MatrixXd& getTaskiProjector() const;
    LinearFunction* getInnerObjectiveFunction() const;
    Variable& getTaskAccelerationVariable() const;
    void doContactForceControl();
    void setReferenceForce(const Eigen::Wrenchd& fRef);

    //------------------------ friendship ------------------------//
protected:
    friend class ISIRController;    //Only the ISIRController should know about the following functions
    void connectToController(ISIRSolver& _solver, const ISIRDynamicFunction& dynamicEquation, bool useReducedProblem, bool useGSHC);
    void disconnectFromController();
    void update();


protected:
    virtual void doGetOutput(Eigen::VectorXd& output) const;

    void addContactPointInModel();
    void removeContactPointInModel();

    virtual void doActivateContactMode();
    virtual void doDeactivateContactMode();
    virtual void doSetFrictionCoeff();
    virtual void doSetMargin();

    virtual void doSetWeight();

    virtual void doActivateAsObjective();
    virtual void doDeactivateAsObjective();
    virtual void doActivateAsConstraint();
    virtual void doDeactivateAsConstraint();

    void doUpdateAccelerationTask();
    void doUpdateTorqueTask();
    void doUpdateForceTask();

    void checkIfConnectedToController() const;

private:
    struct Pimpl;
    boost::shared_ptr<Pimpl> pimpl;

};


/** \} */ // end group task

}

#endif

/**
 * \file ISIRController.h
 * \author Joseph Salini, Mingxing Liu
 *
 * \brief Define the LQP-based controller developped during my PhD thesis with xde framework.
 *
 * The LQP-base controller is explained in my thesis available here: http://hal.archives-ouvertes.fr/tel-00710013/ .
 * Based on a dynamic model of the robot, we have to define tasks and constraints to properly control the system.
 *
 * File history
 * - 14/01/23: Mingxing Liu - Adaptation to generalized smooth hierarchical controller (GSHC).
 */

#ifndef __ISIRCTRL_H__
#define __ISIRCTRL_H__

#include <string>
#include <vector>
#include <iostream>

// ORC INCLUDES
#include "orc/control/Controller.h"
#include "orc/control/Model.h"


// ORCISIR INCLUDES
#include "orcisir/Tasks/ISIRTask.h"
#include "orcisir/Solvers/ISIRSolver.h"
#include "orcisir/Constraints/ISIRConstraint.h"
#include "orcisir/Constraints/GSHCAccelerationConstraint.h"

using namespace orc;

namespace orcisir
{

/** \addtogroup core
 * \{
 */

/** \brief ISIR Controller based on LQP solver for the xde framework.
 *
 */
class ISIRController: public Controller
{
public:

    ISIRController(const std::string& ctrlName, Model& innerModel, ISIRSolver& innerSolver, bool useReducedProblem, bool useGSHC);
    virtual ~ISIRController();

    //getter
    Model&      getModel();
    ISIRSolver& getSolver();
    bool        isUsingReducedProblem();

    //setter
    void setVariableMinimizationWeights(double w_ddq, double w_tau, double w_fc);
    void takeIntoAccountGravity(bool useGrav);

    void writePerformanceInStream(std::ostream& myOstream, bool addCommaAtEnd) const;
    std::string getPerformances() const;

    void addConstraint(orc::LinearConstraint& constraint) const;
    void removeConstraint(orc::LinearConstraint& constraint) const;
    void addConstraint(ISIRConstraint& constraint) const;
    void removeConstraint(ISIRConstraint& constraint) const;

    ISIRTask& createISIRTask(const std::string& name, const Feature& feature, const Feature& featureDes) const;
    ISIRTask& createISIRTask(const std::string& name, const Feature& feature) const;
    ISIRTask& createISIRContactTask(const std::string& name, const PointContactFeature& feature, double mu, double margin) const;

    void setActiveTaskVector();
    void doUpdateAugmentedJacobian();
    void doUpdateProjector();
    std::vector< ISIRTask* >& getActiveTask();
    int getNbActiveTask() const;
    int getTotalActiveTaskDimensions() const;

    void initPriorityMatrix();

    std::pair<Eigen::VectorXd, Eigen::MatrixXd> sortRows(const Eigen::MatrixXd &C, const Eigen::MatrixXd &J);
    void computeProjector(const Eigen::MatrixXd &C, const Eigen::MatrixXd &J, Eigen::MatrixXd& projector);
    void computeTaskiProjector(const Eigen::MatrixXd &J, Eigen::MatrixXd& projector);
    void setTaskProjectors(Eigen::MatrixXd& param);

    orc::CompositeVariable& getTaskVariables();
    orc::EqualZeroConstraintPtr< orcisir::GSHCFunction >      gshcFunction;
    void setGSHCConstraint();


protected:
    virtual void doComputeOutput(Eigen::VectorXd& tau);
    virtual void doAddTask(Task& task);
    virtual void doAddContactSet(const ContactSet& contacts);

protected: // factory
    virtual Task* doCreateTask(const std::string& name, const Feature& feature, const Feature& featureDes) const;
    virtual Task* doCreateTask(const std::string& name, const Feature& feature) const;
    virtual Task* doCreateContactTask(const std::string& name, const PointContactFeature& feature, double mu, double margin) const;


private:
    struct Pimpl;
    boost::shared_ptr<Pimpl> pimpl;
};

/** \} */ // end group core

} //end namespace orcisir



#endif

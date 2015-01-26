/**
 * \file ISIRSolver.h
 * \author Joseph Salini
 *
 * \brief Define the internal solver class that can be used in the ISIR controller.
 *
 * Here, only an abstract class is defined.
 */

#ifndef __ISIRSOLVER_H__
#define __ISIRSOLVER_H__

#include "orc/optim/Solver.h"
#include "orc/optim/Objective.h"
#include "orc/optim/QuadraticFunction.h"
#include "orc/optim/SquaredLinearFunction.h"

#include "orc/control/Model.h"

#include <string>


#include "orcisir/Performances.h"


namespace orcisir
{

/** \addtogroup solver
 * \{
 */

//typedef orc::Objective<orc::SquaredLinearFunction>  SquaredLinearObjective;
//typedef orc::Objective<orc::QuadraticFunction>  QuadraticObjective;

typedef Eigen::Map<Eigen::MatrixXd> MatrixMap;
typedef Eigen::Map<Eigen::VectorXd> VectorMap;

/** \brief A generic abstract class the solvers that can be used in the ISIR Controller.
 *
 * It is based on quadratic solvers.
 *
 * To get a concrete implementation of ISIR solvers, you should call:
 *
 *      - orcisir::OneLevelSolver if we consider that all the tasks registered have the same level of importance (but not necessarily the same weights)
 *      - orcisir::HierarchySolver if we consider that tasks registered can have different level of importance
 */
class ISIRSolver: public orc::Solver
{
public:
    ISIRSolver();
    virtual ~ISIRSolver();

    virtual void printValuesAtSolution();
    virtual std::string toString() const;

//    virtual void addObjective(SquaredLinearObjective& obj);
//    virtual void removeObjective(SquaredLinearObjective& obj);
//    virtual void setObjectiveLevel(SquaredLinearObjective& obj, int level)  = 0;

    virtual void addObjective(orc::QuadraticObjective& obj);
    virtual void removeObjective(orc::QuadraticObjective& obj);
    virtual void setObjectiveLevel(orc::QuadraticObjective& obj, int level)  = 0;

    void addConstraint(orc::LinearConstraint& constraint);
    void removeConstraint(orc::LinearConstraint& constraint);

    void writePerformanceInStream(std::ostream& myOstream, bool addCommaAtEnd);


protected:

    virtual void doPrepare();
    virtual void doSolve()  = 0;
    virtual void doConclude();

    // function in doPrepare
    virtual void prepareMatrices();
    virtual void updateObjectiveEquations()  = 0;
    virtual void updateConstraintEquations() = 0;

    //function in doSolve
    void reduceConstraints(const Eigen::MatrixXd& A, const Eigen::VectorXd& b, Eigen::MatrixXd& Ar, Eigen::VectorXd& br, double tolerance=1e-6);

protected:

    //std::vector<SquaredLinearObjective*>    _objectives;
    std::vector<orc::QuadraticObjective*>    _objectives;

    std::vector<orc::LinearConstraint*> _equalityConstraints;         //< set of constraints
    Eigen::MatrixXd _A;
    Eigen::VectorXd _b;
    // for equality constraint, which will grow over levels, or when they are reduced
    Eigen::MatrixXd _Atotal;
    Eigen::VectorXd _btotal;
    int ne;

    std::vector<orc::LinearConstraint*> _inequalityConstraints;         //< set of constraints
    Eigen::MatrixXd _G;
    Eigen::VectorXd _h;
    int ni;

    Eigen::VectorXd Xsolution;


    PerformanceRecorder prepareRecorder;
    PerformanceRecorder solveRecorder;

};


/** \} */ // end group solver

}// namespace orcisir

#endif

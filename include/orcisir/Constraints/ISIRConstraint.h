/**
 * \file ISIRConstraint.h
 * \author Joseph Salini
 *
 * \brief Define base class that can be used as constraints in ISIR controller.
 */

#ifndef __ISIRCONSTRAINT_H__
#define __ISIRCONSTRAINT_H__


#include "orc/optim/LinearFunction.h"
#include "orc/control/Model.h"
#include "orc/optim/Variable.h"

#include "orc/optim/Constraint.h"


namespace orcisir
{

/** \addtogroup constraint
 * \{
 */


///////// FOR DYNAMIC EQUATION FUNCTION!!!
/** \brief Create a linear function that represents the dynamic equation of motion.
 *
 * The equation of motion is:
 * \f[
 *      \M \ddq + \n + \g = S \torque - \J_c\tp \force_c
 * \f]
 *
 * So given the variable of our problem \f$ \x = \begin{bmatrix}  \ddq\tp & \torque\tp & \force_c\tp\end{bmatrix}\tp \f$ It returns an equation of the form:
 *
 * \f{align*}{
 *      \A \x + \b &= \vec{0} & &\Leftrightarrow & \begin{bmatrix} \M  &  -S  & \J_c\tp \end{bmatrix} . \begin{bmatrix}  \ddq \\ \torque \\ \force_c\end{bmatrix} + [ \n + \g ] = \vec{0}
 * \f}
 */
class ISIRDynamicFunction: public orc::LinearFunction
{
    public:
        typedef orc::LinearFunction  functionType_t;     //< alias on the type of the mother class. Needed to duplicate the function tree.

        ISIRDynamicFunction(const orc::Model& model);
        ~ISIRDynamicFunction();

        void takeIntoAccountGravity(bool useGrav);

        const Eigen::MatrixXd& getInertiaMatrixInverseJchiT() const;
        const Eigen::VectorXd& getInertiaMatrixInverseLinNonLinGrav() const;
        orc::Variable&         getActionVariable() const;

    protected:
        void updateJacobian() const;
        void updateb()        const;
        void buildA();

        virtual void doUpdateInputSizeBegin();
        virtual void doUpdateInputSizeEnd();


    private: // Forbid copy
        ISIRDynamicFunction(ISIRDynamicFunction&);
        ISIRDynamicFunction& operator= (const ISIRDynamicFunction&);

        static orc::Variable& createDEVariable(const orc::Model& model);

        void invalidateReducedProblemData(int timestamp);

    private:
        struct Pimpl;
        boost::shared_ptr<Pimpl> pimpl;

};





class ISIRConstraint
{
public:
    ISIRConstraint()
        : _constraint() {};

    virtual ~ISIRConstraint() {};

    orc::LinearConstraint& getConstraint()
    {
        return *_constraint.get();
    }

protected:
    friend class ISIRController;    //Only the ISIRController should know about the following functions
    virtual void connectToController(const ISIRDynamicFunction& dynamicEquation, bool useReducedProblem) = 0;
    virtual void disconnectFromController() = 0;

    boost::shared_ptr< orc::Constraint<orc::LinearFunction> >   _constraint;
};




/** \} */ // end group constraint

}



#endif

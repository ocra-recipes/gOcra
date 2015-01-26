/**
 * \file TorqueLimitConstraint.h
 * \author Joseph Salini
 *
 * \brief Define torque limit constraint for ISIR controller.
 */

#ifndef __TORQUELIMITCONSTRAINT_H__
#define __TORQUELIMITCONSTRAINT_H__


#include "orcisir/Constraints/ISIRConstraint.h"


namespace orcisir
{

/** \addtogroup constraint
 * \{
 */



/** \brief Create a linear function that represents the torque limit function.
 *
 * The torque limit inequality is:
 * \f[
 *      - \torque_{max} < \torque < \torque_{max}
 * \f]
 *
 * It returns an equation of the form:
 *
 * \f{align*}{
 *      \A \x + \b &> \vec{0} & &\Leftrightarrow & \begin{bmatrix} - \Id{} \\ \Id{} \end{bmatrix} .  \torque  + \begin{bmatrix} \torque_{max} \\ \torque_{max} \end{bmatrix} &> \vec{0}
 * \f}
 */
class TorqueLimitFunction: public orc::LinearFunction
{
public:
    typedef orc::LinearFunction  functionType_t;     //< alias on the type of the mother class. Needed to duplicate the function tree.

    TorqueLimitFunction(const orc::Model& model);
    ~TorqueLimitFunction();

    void  setTorqueLimits(const Eigen::VectorXd& torqueLimits);
    const Eigen::VectorXd& getTorqueLimits() const;

protected:
    void updateJacobian() const;
    void updateb()        const;

    Eigen::VectorXd       _torqueLimits;
    int                   _torqueDim;

private: // Forbid copy
    TorqueLimitFunction(TorqueLimitFunction&);
    TorqueLimitFunction& operator= (const TorqueLimitFunction&);
};






class TorqueLimitConstraint: public orc::LinearConstraint
{
public:
    TorqueLimitConstraint(const orc::Model& model)
        : orc::LinearConstraint(createTorqueLimiFunction(model)) {}; //GREATER_THAN_ZERO

    TorqueLimitConstraint(const orc::Model& model, const Eigen::VectorXd& torqueLimits)
        : orc::LinearConstraint(createTorqueLimiFunction(model)) //GREATER_THAN_ZERO
    {
        _torqueLimitFunction->setTorqueLimits(torqueLimits);
    };

    virtual ~TorqueLimitConstraint()
    {
        delete _torqueLimitFunction;
    }

    void  setTorqueLimits(const Eigen::VectorXd& torqueLimits) {_torqueLimitFunction->setTorqueLimits(torqueLimits);};
    const Eigen::VectorXd& getTorqueLimits() const {return _torqueLimitFunction->getTorqueLimits();};

private:
    TorqueLimitFunction* createTorqueLimiFunction(const orc::Model& model)
    {
        _torqueLimitFunction = new TorqueLimitFunction(model);
        return _torqueLimitFunction;
    }

    TorqueLimitFunction* _torqueLimitFunction;
};

/** \} */ // end group constraint

}


#endif




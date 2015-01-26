/**
 * \file GSHCAccelerationConstraint.cpp
 * \author Mingxing Liu
 *
 * \brief ddq = sum_i projector_i*ddq_i.
 */

#include "orcisir/Constraints/GSHCAccelerationConstraint.h"


using namespace orcisir;

struct GSHCFunction::Pimpl
{
    const orc::Model&                       _model;
    CompositeVariable&                          _var;
    std::vector< ISIRTask* >&               _activeTask;

    Pimpl(const orc::Model& model, std::vector< ISIRTask* >& activeTask, CompositeVariable& var)
        : _model(model)
        , _var("GSHC.var",var)
        , _activeTask(activeTask)
    {

    }
};


GSHCFunction::GSHCFunction(const orc::Model& model, std::vector< ISIRTask* >& activeTask, CompositeVariable& var)
    : orc::NamedInstance("GSHC Function")
    , orc::AbilitySet(orc::PARTIAL_X)
    , orc::CoupledInputOutputSize(false)
    , pimpl(new Pimpl(model, activeTask, var))
    , LinearFunction(var, model.nbDofs())
{
    /* The function is on the form:
     *      ddq  - sum_i projector_i * ddq_i = 0
     *
     * The associated constraint is:
     *      A ddq + b = 0
     */
    pimpl->_model.connect<orc::EVT_CHANGE_VALUE>(*this, &GSHCFunction::invalidateAll); // to update A (jacobian) at each time step
//    pimpl->_model.connect<orc::EVT_CHANGE_VALUE>(*this, &GSHCFunction::invalidateb); // not necessary because b is constant

    buildA();
    _b = VectorXd::Zero(pimpl->_model.nbDofs());
}

//void GSHCFunction::setActiveTaskAndVariables(std::vector< ISIRTask* >& activeTask, orc::Variable& var)
//{
//    pimpl->_activeTask = activeTask;
//    pimpl->_var.clear();
//    pimpl->_var.add(var);

//    buildA();
//}

void GSHCFunction::deleteVariables()
{
//    std::cout<<pimpl->_var.getSize()<<std::endl;
//    std::cout<<pimpl->_var.getNumParenthoods()<<std::endl;
    pimpl->_var.clear();
}


void GSHCFunction::buildA()
{
    int nDofs = pimpl->_model.nbDofs();

    Eigen::MatrixXd Iddq = Eigen::MatrixXd::Identity(nDofs, nDofs);

    _jacobian =  Eigen::MatrixXd::Zero(nDofs, pimpl->_var.getSize());
    _jacobian.topLeftCorner(nDofs,nDofs)    = Iddq;

}

void GSHCFunction::updateJacobian() const
{
    int nDofs = pimpl->_model.nbDofs();

    if (pimpl->_var.getSize()==(nDofs*(pimpl->_activeTask.size()+1)))
    {
        for (int i=0; i<pimpl->_activeTask.size(); ++i)
        {
            _jacobian.block(0,nDofs*(i+1),nDofs, nDofs) =  -pimpl->_activeTask[i]->getProjector();
        }
    }

}

void GSHCFunction::updateb() const
{
//    _b = VectorXd::Zero(pimpl->_model.nbDofs());
}

/** Destructor
 *
 * It is disconnected from the model.
 */
GSHCFunction::~GSHCFunction()
{
    pimpl->_model.disconnect<orc::EVT_CHANGE_VALUE>(*this, &GSHCFunction::invalidateAll);
//    pimpl->_model.disconnect<orc::EVT_CHANGE_VALUE>(*this, &GSHCFunction::invalidateb);
    deleteVariables();
//    delete &getVariable();
}


/** Do when input size changes, before.
 *
 * By overloading this function, it allows linear function modification when input size changes.
 * It does nothing actually.
 */
void GSHCFunction::doUpdateInputSizeBegin()
{
    //do nothing : this overload allows to resize
}

/** Do when input size changes, after.
 *
 * By overloading this function, it allows linear function modification when input size changes.
 * It does nothing actually.
 */
void GSHCFunction::doUpdateInputSizeEnd()
{
    updateJacobian();
}

//GSHCAccelerationConstraint::GSHCAccelerationConstraint(const orc::Model& model, std::vector< ISIRTask* >& activeTask)
//    : _model(model)
//    , _is_connected(false)
//    , _activeTask(activeTask)
//    , _var("taskVariables", model.getAccelerationVariable())
//    , ISIRConstraint()
//{
//    for (int i=0; i<_activeTask.size(); ++i)
//    {
//        _var.add(_activeTask[i]->getTaskAccelerationVariable());
//    }
//}


//void GSHCAccelerationConstraint::connectToController(const ISIRDynamicFunction& dynamicEquation, bool useReducedProblem)
//{
//    orc::LinearFunction* f = NULL;
//    f = createGSHCFunction(_model, _activeTask, _var);

//    _constraint.reset(new orc::LinearConstraint(f));
//    _is_connected = true;

//}

//void GSHCAccelerationConstraint::disconnectFromController()
//{
//    _is_connected = false;
//}


//GSHCFunction* GSHCAccelerationConstraint::createGSHCFunction(const orc::Model& model, std::vector< ISIRTask* >& activeTask, CompositeVariable& var)
//{
//    _GSHCFunctionFunction = new GSHCFunction(model, activeTask, var);
//    return _GSHCFunctionFunction;
//}

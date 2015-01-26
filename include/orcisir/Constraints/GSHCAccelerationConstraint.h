/**
 * \file GSHCAccelerationConstraint.h
 * \author Mingxing Liu
 *
 * \brief ddq = sum_i projector_i*ddq_i.
 */

#ifndef __GSHCACCELARTIONCONSTRAINT_H__
#define __GSHCACCELARTIONCONSTRAINT_H__


#include "orcisir/Constraints/ISIRConstraint.h"
#include "orcisir/Tasks/ISIRTask.h"

namespace orcisir
{

/** \addtogroup constraint
 * \{
 */


class GSHCFunction: public orc::LinearFunction
{
    public:
        typedef orc::LinearFunction  functionType_t;     //< alias on the type of the mother class. Needed to duplicate the function tree.

        GSHCFunction(const orc::Model& m, std::vector< ISIRTask* >& activeTask, orc::CompositeVariable& var);
        ~GSHCFunction();
        void                deleteVariables();

    private: // Forbid copy
        GSHCFunction(GSHCFunction&);
        GSHCFunction& operator= (const GSHCFunction&);


    protected:
        struct Pimpl;
        boost::shared_ptr<Pimpl> pimpl;


        void                buildA();
        void                updateb() const;
        void                updateJacobian() const;

        virtual void doUpdateInputSizeBegin();
        virtual void doUpdateInputSizeEnd();
};



//class GSHCAccelerationConstraint: public ISIRConstraint
//{
//public:
//    GSHCAccelerationConstraint(const orc::Model& model, std::vector< ISIRTask* >& activeTask);
//    virtual ~GSHCAccelerationConstraint() {};
////    void setTaskVariables(orc::Variable& var);


//protected:
//    virtual void connectToController(const ISIRDynamicFunction& dynamicEquation, bool useReducedProblem);
//    virtual void disconnectFromController();

//private:
//    GSHCFunction* createGSHCFunction(const orc::Model& model, std::vector< ISIRTask* >& activeTask, orc::CompositeVariable& var);

//    GSHCFunction* _GSHCFunctionFunction;
//    const orc::Model&           _model;
//    bool                        _is_connected;
//    std::vector< ISIRTask* >&   _activeTask;
//    orc::CompositeVariable      _var;
//};



/** \} */ // end group constraint

}
#endif

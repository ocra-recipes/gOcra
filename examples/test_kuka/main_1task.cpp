
#include <iostream>

#include <Eigen/Eigen>

#include "kukafixed.h"
//#include "kukafree.h"
#include "orcisir/OrthonormalFamily.h"

#include "orcisir/ISIRController.h"
#include "orcisir/Solvers/OneLevelSolver.h"

#include "orc/control/Feature.h"
#include "orc/control/FullState.h"
#include "orc/control/ControlFrame.h"
#include "orc/control/ControlEnum.h"

#include "orcisir/Features/ISIRFeature.h"

#include "orcisir/Constraints/ISIRConstraint.h"
#include "orcisir/Constraints/GSHCAccelerationConstraint.h"

#include <stdlib.h>
#include <sys/time.h>
int main(int argc, char** argv)
{
    timespec ts;
    timeval start, end;
    double elapsed_ms = 0.0;
    std::cout<<"=============================================================================================================\n";
    //SET CONTROLLER PARAMETERS
    std::cout<<"SET PARAMETERS\n";
    bool useReducedProblem = false;
    bool useGSHC = true;

    //INITIALIZE ISIR MODEL, CONTROLLER
    std::cout<<"INITIALIZE ISIR MODEL, CONTROLLER\n";
    kukafixed                             model("kuka");

    orcisir::OneLevelSolverWithQuadProg   solver;

    orcisir::ISIRController               ctrl("myCtrl", model, solver, useReducedProblem, useGSHC);

    ctrl.takeIntoAccountGravity(false);


    // INITIALIZE ISIR MODEL, CONTROLLER
    std::cout<<"INITIALIZE TASKS\n";

//================ FULL STATE ==============================================================================//
    orc::FullModelState   FMS("torqueTask.FModelState", model, orc::FullState::INTERNAL); //INTERNAL, FREE_FLYER
    orc::FullTargetState  FTS("torqueTask.FTargetState", model, orc::FullState::INTERNAL);
    orc::FullStateFeature feat("torqueTask", FMS);
    orc::FullStateFeature featDes("torqueTask.Des", FTS);

    FTS.set_q(Eigen::VectorXd::Constant(model.nbInternalDofs(), 0.4));
    //FTS.set_qddot(Eigen::VectorXd::Constant(model.nbInternalDofs(), 0.1));

    orcisir::ISIRTask& accTask = ctrl.createISIRTask("accTask", feat, featDes);
    accTask.initAsAccelerationTask();
    ctrl.addTask(accTask);
    accTask.activateAsObjective();
    accTask.setStiffness(4);
    accTask.setDamping(4);
    accTask.setWeight(0.000001);



    //    if(useGSHC)
    //    {
            ctrl.setActiveTaskVector();

            // SET TASK PRIORITIES, COMPUTE TASK PROJECTORS
            int nt = ctrl.getNbActiveTask();

            MatrixXd param_priority(nt,nt);
        //    std::vector< orcisir::ISIRTask* >& activeTask = ctrl.getActiveTask();
            param_priority<<0;
            ctrl.setTaskProjectors(param_priority);

            ctrl.setGSHCConstraint();

    //    }

    VectorXd q      = Eigen::VectorXd::Constant(model.nbInternalDofs(), 0.1);
    VectorXd dq     = Eigen::VectorXd::Zero(model.nbInternalDofs());
    VectorXd tau    = Eigen::VectorXd::Zero(model.nbInternalDofs());
    double dt = 0.01;

    model.setJointPositions(q);
    model.setJointVelocities(dq);


    //SIMULATE
    std::cout<<"SIMULATE\n";
    for (int i=0; i<1000; i++)
    {
//        std::cout<<i<<"\n";
        if (i%10==0)
        {
            FTS.set_q(Eigen::VectorXd::Constant(model.nbInternalDofs(), 0.39+0.001*(rand()%10)));

        }
        model.setJointPositions(q);
        model.setJointVelocities(dq);


        ctrl.setTaskProjectors(param_priority);
        ctrl.doUpdateProjector();
        gettimeofday(&start, NULL);
        ctrl.computeOutput(tau);    //compute tau
        gettimeofday(&end, NULL);
        elapsed_ms += (end.tv_sec - start.tv_sec)*1000.0;///CLOCKS_PER_SEC;
        //VectorXd ddq = model.getAccelerationVariable().getValue();
        //VectorXd ddq = model.getInertiaMatrixInverse() * ( tau - model.getNonLinearTerms() - model.getLinearTerms() - model.getGravityTerms() );
        VectorXd ddq = model.getInertiaMatrixInverse() * ( tau - model.getNonLinearTerms() - model.getLinearTerms());

        dq += ddq * dt;
        q  += dq  * dt;


//        if (i==599||i==999)
//        {
//            std::cout<<"tau: "<<tau.transpose()<<"\n";
//            std::cout<<"ddq: "<<ddq.transpose()<<"\n";
//            std::cout<<"q  : "<<model.getJointPositions().transpose()<<"\n";
//            std::cout<<"pos EE: "<< model.getSegmentPosition(7).getTranslation().transpose()<<"\n";
//            std::cout<<"ori EE: "<< model.getSegmentPosition(7).getRotation()<<"\n";
//        }
        //std::cout<<solver.toString()<<"\n";
        std::cout<<elapsed_ms/(i+1)<<",";
    }

    std::cout<<std::endl;
    std::cout<<"END OF MAIN\n";
    return 0;
}

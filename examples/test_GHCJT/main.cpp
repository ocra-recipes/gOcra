
#include <iostream>
#include <map>

#include <Eigen/Eigen>

#include "Model3T.h"
#include "orcisir/OrthonormalFamily.h"

#include "orcisir/GHCJTController.h"
#include "orcisir/Solvers/OneLevelSolver.h"
#include "orc/control/Feature.h"
#include "orc/control/FullState.h"
#include "orc/control/ControlFrame.h"
#include "orc/control/ControlEnum.h"
#include "orcisir/Constraints/ISIRConstraint.h"



int main(int argc, char** argv)
{
    std::cout<<"SET PARAMETERS\n";

    bool useGSHC = true;
    bool useGrav = true;

    orcisir::OneLevelSolverWithQuadProg   internalSolver;
    
    // INITIALIZE ISIR MODEL, CONTROLLER & TASK MANAGER
    std::cout<<"INITIALIZE ISIR MODEL, CONTROLLER\n";
    Model3T                         model("Model3T");
    orcisir::GHCJTController         ctrl("myCtrl", model, internalSolver, useGSHC, useGrav);


    //CREATE SOME TASKS
    std::cout<<"CREATE SOME TASKS\n";
    orc::FullModelState   FMS("fullTask.FModelState" , model, orc::FullState::INTERNAL); //INTERNAL, FREE_FLYER
    orc::FullTargetState  FTS("fullTask.FTargetState", model, orc::FullState::INTERNAL);
    orc::FullStateFeature feat("fullTask.feat", FMS);
    orc::FullStateFeature featDes("fullTask.featDes", FTS);
    FTS.set_q(Eigen::VectorXd::Zero(model.nbInternalDofs()));

    orcisir::GHCJTTask& accTask = ctrl.createGHCJTTask("fullTask", feat, featDes);

    ctrl.addTask(accTask);
    accTask.activateAsObjective();
    accTask.setStiffness(9);
    accTask.setDamping(6);
    accTask.setWeight(1.);

    orc::SegmentFrame        SF("frameTask.SFrame", model, "Model3T.segment_3", Eigen::Displacementd());
    orc::TargetFrame         TF("frameTask.TFrame", model);
    orc::PositionFeature feat2("frameTask.feat", SF, orc::XYZ);
    orc::PositionFeature featDes2("frameTask.featDes", TF, orc::XYZ);

    TF.setPosition(Eigen::Displacementd(0.2,0.3,0.4));
    TF.setVelocity(Eigen::Twistd());
    TF.setAcceleration(Eigen::Twistd());


    orcisir::GHCJTTask& accTask2 = ctrl.createGHCJTTask("frameTask", feat2, featDes2);

    ctrl.addTask(accTask2);
    accTask2.activateAsObjective();
    accTask2.setStiffness(9);
    accTask2.setDamping(6);
    accTask2.setWeight(1);

    ctrl.setActiveTaskVector();

    // SET TASK PRIORITIES, COMPUTE TASK PROJECTORS
    int nt = ctrl.getNbActiveTask();

    MatrixXd param_priority(nt,nt);
//        std::vector< orcisir::GHCJTTask* >& activeTask = ctrl.getActiveTask();

    param_priority<<0,1,0,0;
    ctrl.setTaskProjectors(param_priority);


    //SIMULATE
    VectorXd q      = Eigen::VectorXd::Constant(model.nbInternalDofs(), 0.1);
    VectorXd dq     = Eigen::VectorXd::Zero(model.nbInternalDofs());
    VectorXd tau    = Eigen::VectorXd::Zero(model.nbInternalDofs());
    VectorXd ddq;
    double dt = 0.01;

    std::cout<<"SIMULATE\n";
    for (int i=0; i<1000; i++)
    {
        if (i==600)
        {
            param_priority(0,1)=0;
            param_priority(1,0)=1;
        }
        ctrl.setTaskProjectors(param_priority);
        ctrl.doUpdateProjector();
        ctrl.computeOutput(tau);
        ddq = model.getInertiaMatrixInverse() * ( tau - model.getNonLinearTerms() - model.getLinearTerms() - model.getGravityTerms());

        dq += ddq * dt;
        q  += dq  * dt;



        model.setJointPositions(q);
        model.setJointVelocities(dq);
        if (i==1||i==599||i==999)
        {
            std::cout<<"- -  --- - - - -- - - -- - "<<i<<"\n";

            std::cout<<"pos seg3: "<< model.getSegmentPosition(3).getTranslation().transpose()<<"\n";

        }

    }


    return 0;
}





#include <iostream>

#include <Eigen/Eigen>

#include "kukafixed.h"
//#include "kukafree.h"

#include "orcisir/GHCJTController.h"
#include "orcisir/Solvers/OneLevelSolver.h"

#include "orc/control/Feature.h"
#include "orc/control/FullState.h"
#include "orc/control/ControlFrame.h"
#include "orc/control/ControlEnum.h"

//#include "orcisir/Features/ISIRFeature.h"

//#include "orcisir/Constraints/ISIRConstraint.h"


#include <stdlib.h>
#include <sys/time.h>

int main(int argc, char** argv)
{
//    timespec ts;
//    timeval start, end;
//    double elapsed_ms = 0.0;

    //SET CONTROLLER PARAMETERS
    std::cout<<"SET PARAMETERS\n";

    bool useGSHC = true;
    bool useGrav = true;

    //INITIALIZE ISIR MODEL, CONTROLLER
    std::cout<<"INITIALIZE ISIR MODEL, CONTROLLER\n";
    kukafixed                             model("kuka");

    orcisir::OneLevelSolverWithQuadProg   solver;

    orcisir::GHCJTController               ctrl("myCtrl", model, solver, useGSHC, useGrav);


    std::cout<<"INITIALIZE TASKS\n";

//================ FULL STATE ==============================================================================//
    orc::FullModelState   FMS("torqueTask.FModelState", model, orc::FullState::INTERNAL); //INTERNAL, FREE_FLYER
    orc::FullTargetState  FTS("torqueTask.FTargetState", model, orc::FullState::INTERNAL);
    orc::FullStateFeature feat("torqueTask", FMS);
    orc::FullStateFeature featDes("torqueTask.Des", FTS);

    VectorXd qdes(7);
    qdes<<0.2,-0.25,0.2,1.3,0.2,-1.2,0.2;
    FTS.set_q(qdes);
//    FTS.set_q(Eigen::VectorXd::Constant(model.nbInternalDofs(), 0.25));

    orcisir::GHCJTTask& accTask = ctrl.createGHCJTTask("accTask", feat, featDes);
//    accTask.initAsAccelerationTask();
    ctrl.addTask(accTask);
    accTask.activateAsObjective();
    VectorXd kp_pos(7);
    kp_pos<<60,90,30,20,9,9,9;
    VectorXd kd_pos(7);
    kd_pos<<0.6,0.9,0.3,0.2,0.1,0.1,0.1;
    accTask.setStiffness(kp_pos);
    accTask.setDamping(kd_pos);




//================ FRAME ==============================================================================//
    orc::SegmentFrame        SF("frame.SFrame", model, "kuka.07", Eigen::Displacementd());
    orc::TargetFrame         TF("frame.TFrame", model);
    orc::PositionFeature feat2("frame", SF, orc::XYZ);
    orc::PositionFeature featDes2("frame.Des", TF, orc::XYZ);

    TF.setPosition(Eigen::Displacementd(0.3,0.0,0.5));//0.3,0.0,0.5//0.478093, 0.196918,  0.63832
    TF.setVelocity(Eigen::Twistd());
    TF.setAcceleration(Eigen::Twistd());

    orcisir::GHCJTTask& accTask2 = ctrl.createGHCJTTask("accTask2", feat2, featDes2);
//    accTask2.initAsAccelerationTask();
    ctrl.addTask(accTask2);
    accTask2.activateAsObjective();
    accTask2.setStiffness(30);//30,18 ; 16,12 ; 5,7
    accTask2.setDamping(18);

    ctrl.setActiveTaskVector();
    int nt = ctrl.getNbActiveTask();
    MatrixXd param_priority(nt,nt);
    param_priority<<0,1,0,0;

//    std::vector< orcisir::ISIRTask* >& activeTask = ctrl.getActiveTask();

    VectorXd qini(7);
    qini<<0.0,-0.05,0.0,1.5,0.0,-1.0,0.0;
    VectorXd q      = Eigen::VectorXd::Constant(model.nbInternalDofs(), 0.02);
    q = qini;
    VectorXd dq     = Eigen::VectorXd::Zero(model.nbInternalDofs());
    VectorXd ddq    = Eigen::VectorXd::Zero(model.nbInternalDofs());
    VectorXd tau    = Eigen::VectorXd::Zero(model.nbInternalDofs());
    double dt = 0.001;

    model.setJointPositions(q);
    model.setJointVelocities(dq);

    //SIMULATE
    std::cout<<"SIMULATE\n";

    Vector3d e0;
    Vector3d error,errDot,errorDot;
    Eigen::Displacementd::Rotation3D R,Rdes,Rdes_in_r;
    MatrixXd spaceTransform;
    VectorXd eq,deq;

    int t1 = 30000;

    for (int i=0; i<3*t1; i++)
    {

        if (i==t1)
        {
            param_priority<<0,1,0,0;
            std::cout<<"change priority"<<std::endl;
        }
        if (i==0||i==2*t1)
        {
            param_priority<<0,0,1,0;
            std::cout<<"change priority"<<std::endl;
        }
        ctrl.setTaskProjectors(param_priority);
        ctrl.doUpdateProjector();
//        gettimeofday(&start, NULL);
        ctrl.computeOutput(tau);    //compute tau


        eq = FMS.q() - FTS.q();
        deq = FMS.qdot() - FTS.qdot();

//        e0 = SF.getPosition().getTranslation() - TF.getPosition().getTranslation();
//        e_in_r = SF.getPosition().getRotation().inverse() * e0;
//        R = SF.getPosition().getRotation();
        spaceTransform = SF.getPosition().getRotation().adjoint();
//        error = spaceTransform * e_in_r;
        error = SF.getPosition().getTranslation() - TF.getPosition().getTranslation();
//        std::cout<<"error= "<<error.transpose()<<"\n";


        Rdes_in_r = SF.getPosition().getRotation().inverse() * TF.getPosition().getRotation();
        errDot = SF.getVelocity().getLinearVelocity() - Rdes_in_r.adjoint() * TF.getVelocity().getLinearVelocity();
        errorDot = spaceTransform * errDot;
//        std::cout<<"errorDot= "<<errorDot.transpose()<<"\n";


        const VectorXd f1 = -accTask.getStiffness() * eq
                - accTask.getDamping() * deq;
        const VectorXd f2 = -accTask2.getStiffness() * error
                - accTask2.getDamping() * errorDot;


//        const VectorXd f1 = -accTask.getStiffness() * accTask.getError()
//                - accTask.getDamping() * accTask.getErrorDot();
//        const VectorXd f2 = -accTask2.getStiffness() * accTask2.getError()
//                - accTask2.getDamping() * accTask2.getErrorDot();

        tau = (model.getGravityTerms()).tail(model.nbInternalDofs())
                + accTask.getProjector().transpose()*accTask.getJacobian().transpose() * f1
                + accTask2.getProjector().transpose()*accTask2.getJacobian().transpose() * f2;

//        param_priority<<0,0,1,0;
//        ctrl.setTaskProjectors(param_priority);
//        ctrl.doUpdateProjector();
//        tau = (model.getGravityTerms()).tail(model.nbInternalDofs())
//                + accTask.getProjector().transpose()*accTask.getJacobian().transpose() * f1;


//        param_priority<<0,1,0,0;
//        ctrl.setTaskProjectors(param_priority);
//        ctrl.doUpdateProjector();
//        tau = (model.getGravityTerms()).tail(model.nbInternalDofs())
//                + accTask2.getJacobian().transpose() * f2;

//        tau += accTask.getProjector().bottomRows(model.nbInternalDofs())*accTask.getJacobian().transpose() * f1;
//        tau += accTask2.getJacobian().transpose().bottomRows(model.nbInternalDofs()) * f2;
//        std::cout<<"p1="<<accTask.getProjector()<<std::endl;
//        std::cout<<"p2="<<MatrixXd::Identity(7,7)-accTask2.getJacobian().transpose()*accTask2.getJacobian()<<std::endl;
//          std::cout<<accTask.getProjector().transpose()*accTask.getJacobian().transpose()*f1<<std::endl;

//        gettimeofday(&end, NULL);
//        elapsed_ms += (end.tv_sec - start.tv_sec)*1000.0;///CLOCKS_PER_SEC;

        ddq = model.getInertiaMatrixInverse() * ( tau - model.getNonLinearTerms() - model.getLinearTerms() - model.getGravityTerms() );
//        VectorXd ddq = model.getInertiaMatrixInverse() * ( tau - model.getNonLinearTerms() - model.getLinearTerms());
//        std::cout<<"ddq= "<<ddq.transpose()<<"\n";
        dq += ddq * dt;
        q  += dq  * dt;
        model.setJointPositions(q);
        model.setJointVelocities(dq);

        if (i==(t1-1)||i==(2*t1-1)||i==(3*t1-1))
        {
//            std::cout<<"tau: "<<tau.transpose()<<"\n";
//            std::cout<<"ddq: "<<ddq.transpose()<<"\n";
            std::cout<<"q  : "<<model.getJointPositions().transpose()<<"\n";
            std::cout<<"pos EE: "<< model.getSegmentPosition(7).getTranslation().transpose()<<"\n";
//            std::cout<<"ori EE: "<< model.getSegmentPosition(7).getRotation()<<"\n";
        }
        //std::cout<<solver.toString()<<"\n";
//        std::cout<<elapsed_ms/(i+1)<<",";
    }

    std::cout<<std::endl;
    std::cout<<"END OF MAIN\n";
    return 0;
}

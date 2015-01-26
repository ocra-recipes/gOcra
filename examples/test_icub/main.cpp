
#include <iostream>

#include <Eigen/Eigen>

#include "icubfixed.h"

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
    bool useGSHC = false;

    //INITIALIZE ISIR MODEL, CONTROLLER
    std::cout<<"INITIALIZE ISIR MODEL, CONTROLLER\n";
    icubfixed                             model("icub");

    orcisir::OneLevelSolverWithQuadProg   solver;

    orcisir::ISIRController               ctrl("myCtrl", model, solver, useReducedProblem, useGSHC);

    ctrl.takeIntoAccountGravity(false);


    // INITIALIZE ISIR MODEL, CONTROLLER
    std::cout<<"INITIALIZE TASKS\n";

//================ FULL STATE ==============================================================================//
    orc::FullModelState*   FMS;
    FMS = new orc::FullModelState("torqueTask.FModelState", model, orc::FullState::INTERNAL); //INTERNAL, FREE_FLYER
    orc::FullTargetState*  FTS;
    FTS = new orc::FullTargetState("torqueTask.FTargetState", model, orc::FullState::INTERNAL);
    orc::FullStateFeature feat("torqueTask", *FMS);
    orc::FullStateFeature featDes("torqueTask.Des", *FTS);

    FTS->set_q(Eigen::VectorXd::Constant(model.nbInternalDofs(), 0.0));
    //FTS.set_qddot(Eigen::VectorXd::Constant(model.nbInternalDofs(), 0.1));

    orcisir::ISIRTask& accTask = ctrl.createISIRTask("accTask", feat, featDes);
    accTask.initAsAccelerationTask();
    ctrl.addTask(accTask);
    accTask.activateAsObjective();
    accTask.setStiffness(4);
    accTask.setDamping(4);
    accTask.setWeight(0.000001);



//================ FRAME ==============================================================================//
    orc::SegmentFrame*        SF;
    SF = new orc::SegmentFrame("frame.SFrame", model, "icub.r_hand", Eigen::Displacementd());
    orc::TargetFrame*         TF;
    TF = new orc::TargetFrame("frame.TFrame", model);
    orc::PositionFeature feat2("frame", *SF, orc::XYZ);
    orc::PositionFeature featDes2("frame.Des", *TF, orc::XYZ);

    TF->setPosition(Eigen::Displacementd(0.0,0.2,0.1));
    TF->setVelocity(Eigen::Twistd());
    TF->setAcceleration(Eigen::Twistd());

    orcisir::ISIRTask& accTask2 = ctrl.createISIRTask("accTask2", feat2, featDes2);
    accTask2.initAsAccelerationTask();
    ctrl.addTask(accTask2);
    accTask2.activateAsObjective();
    accTask2.setStiffness(70);
    accTask2.setDamping(30);
    accTask2.setWeight(100);

//================ FRAME2 ==============================================================================//
    orc::SegmentFrame*        SF2;
    SF2 = new orc::SegmentFrame("frame2.SFrame", model, "icub.l_hand", Eigen::Displacementd());
    orc::TargetFrame*         TF2;
    TF2 = new orc::TargetFrame("frame2.TFrame", model);
    orc::PositionFeature feat3("frame2", *SF2, orc::XYZ);
    orc::PositionFeature featDes3("frame2.Des", *TF2, orc::XYZ);

    TF2->setPosition(Eigen::Displacementd(0.0,-0.2,0.1));
    TF2->setVelocity(Eigen::Twistd());
    TF2->setAcceleration(Eigen::Twistd());

    orcisir::ISIRTask& accTask3 = ctrl.createISIRTask("accTask3", feat3, featDes3);
    accTask3.initAsAccelerationTask();
    ctrl.addTask(accTask3);
    accTask3.activateAsObjective();
    accTask3.setStiffness(70);
    accTask3.setDamping(30);
    accTask3.setWeight(100);

////================ FRAME3 ==============================================================================//
//    orc::SegmentFrame        SF3("frame3.SFrame", model, "icub.head", Eigen::Displacementd());
//    orc::TargetFrame         TF3("frame3.TFrame", model);
//    orc::PositionFeature feat4("frame3", SF3, orc::XYZ);
//    orc::PositionFeature featDes4("frame3.Des", TF3, orc::XYZ);

//    TF3.setPosition(Eigen::Displacementd(0.,0.,1.2));
//    TF3.setVelocity(Eigen::Twistd());
//    TF3.setAcceleration(Eigen::Twistd());

//    orcisir::ISIRTask& accTask4 = ctrl.createISIRTask("accTask4", feat4, featDes4);
//    accTask4.initAsAccelerationTask();
//    ctrl.addTask(accTask4);
//    accTask4.activateAsObjective();
//    accTask4.setStiffness(4);
//    accTask4.setDamping(4);
//    accTask4.setWeight(1);

////================ FRAME4 ==============================================================================//
//    orc::SegmentFrame        SF4("frame4.SFrame", model, "icub.waist", Eigen::Displacementd());
//    orc::TargetFrame         TF4("frame4.TFrame", model);
//    orc::PositionFeature feat5("frame4", SF4, orc::XYZ);
//    orc::PositionFeature featDes5("frame4.Des", TF4, orc::XYZ);

//    TF4.setPosition(Eigen::Displacementd(0.01,0.,1.0));
//    TF4.setVelocity(Eigen::Twistd());
//    TF4.setAcceleration(Eigen::Twistd());

//    orcisir::ISIRTask& accTask5 = ctrl.createISIRTask("accTask5", feat5, featDes5);
//    accTask5.initAsAccelerationTask();
//    ctrl.addTask(accTask5);
//    accTask5.activateAsObjective();
//    accTask5.setStiffness(4);
//    accTask5.setDamping(4);
//    accTask5.setWeight(1);

////================ FRAME4 ==============================================================================//
//    orc::SegmentFrame        SF5("frame5.SFrame", model, "icub.l_foot", Eigen::Displacementd());
//    orc::TargetFrame         TF5("frame5.TFrame", model);
//    orc::PositionFeature feat6("frame5", SF5, orc::XYZ);
//    orc::PositionFeature featDes6("frame5.Des", TF5, orc::XYZ);

//    TF5.setPosition(Eigen::Displacementd(0.01,0.,0.0));
//    TF5.setVelocity(Eigen::Twistd());
//    TF5.setAcceleration(Eigen::Twistd());

//    orcisir::ISIRTask& accTask6 = ctrl.createISIRTask("accTask6", feat6, featDes6);
//    accTask6.initAsAccelerationTask();
//    ctrl.addTask(accTask6);
//    accTask6.activateAsObjective();
//    accTask6.setStiffness(4);
//    accTask6.setDamping(4);
//    accTask6.setWeight(1);
////================ FRAME4 ==============================================================================//
//    orc::SegmentFrame        SF6("frame6.SFrame", model, "icub.r_foot", Eigen::Displacementd());
//    orc::TargetFrame         TF6("frame6.TFrame", model);
//    orc::PositionFeature feat7("frame6", SF6, orc::XYZ);
//    orc::PositionFeature featDes7("frame6.Des", TF6, orc::XYZ);

//    TF6.setPosition(Eigen::Displacementd(-0.01,0.,0.0));
//    TF6.setVelocity(Eigen::Twistd());
//    TF6.setAcceleration(Eigen::Twistd());

//    orcisir::ISIRTask& accTask7 = ctrl.createISIRTask("accTask7", feat7, featDes7);
//    accTask7.initAsAccelerationTask();
//    ctrl.addTask(accTask7);
//    accTask7.activateAsObjective();
//    accTask7.setStiffness(4);
//    accTask7.setDamping(4);
//    accTask7.setWeight(1);

//    ctrl.setActiveTaskVector();

//    // SET TASK PRIORITIES, COMPUTE TASK PROJECTORS
//    int nt = ctrl.getNbActiveTask();

//    MatrixXd param_priority(nt,nt);
//    param_priority<<0,1,0,0,0,0,0,
//            0,0,0,0,0,0,0,
//            0,0,0,0,0,0,0,
//            0,0,0,0,0,0,0,
//            0,0,0,0,0,0,0,
//            0,0,0,0,0,0,0,
//            0,0,0,0,0,0,0;
//    ctrl.setTaskProjectors(param_priority);
//    ctrl.setGSHCConstraint();



    VectorXd q      = Eigen::VectorXd::Constant(model.nbInternalDofs(), 0.0);
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
//        if (i%10==0)
//        {
//            FTS.set_q(Eigen::VectorXd::Constant(model.nbInternalDofs(), 0.39+0.001*(rand()%10)));
//            TF.setPosition(Eigen::Displacementd(0.19+0.001*(rand()%10),0.29+0.001*(rand()%10),0.39+0.001*(rand()%10)));
//            TF2.setPosition(Eigen::Displacementd(-0.19-0.001*(rand()%10),0.29+0.001*(rand()%10),0.39+0.001*(rand()%10)));
//            TF3.setPosition(Eigen::Displacementd(0.0,0.0,1.2+0.001*(rand()%10)));
//            TF4.setPosition(Eigen::Displacementd(0.01,0.0,1.+0.001*(rand()%10)));
//            TF5.setPosition(Eigen::Displacementd(0.01+0.001*(rand()%10),0.0,0.0));
//            TF6.setPosition(Eigen::Displacementd(-0.01+0.001*(rand()%10),0.0,0.0));

//        }
        model.setJointPositions(q);
        model.setJointVelocities(dq);
//        if (i==600)
//        {
//            param_priority(0,1)=0;
//            param_priority(1,0)=1;


//            std::cout<<"change priority"<<std::endl;
//        }

        //ctrl.setTaskProjectors(param_priority);
        //ctrl.doUpdateProjector();
        //gettimeofday(&start, NULL);
        ctrl.computeOutput(tau);    //compute tau
        //gettimeofday(&end, NULL);
        //elapsed_ms += (end.tv_sec - start.tv_sec)*1000.0;///CLOCKS_PER_SEC;
        //VectorXd ddq = model.getAccelerationVariable().getValue();
        //VectorXd ddq = model.getInertiaMatrixInverse() * ( tau - model.getNonLinearTerms() - model.getLinearTerms() - model.getGravityTerms() );
        VectorXd ddq = model.getInertiaMatrixInverse() * ( tau - model.getNonLinearTerms() - model.getLinearTerms());

        dq += ddq * dt;
        q  += dq  * dt;


        if (i==599||i==999)
        {
//            std::cout<<"tau: "<<tau.transpose()<<"\n";
//            std::cout<<"ddq: "<<ddq.transpose()<<"\n";
            std::cout<<"q  : "<<model.getJointPositions().transpose()<<"\n";
            std::cout<<"pos rhand: "<<SF->getPosition().getTranslation().transpose()<<"\n";
            std::cout<<"pos lhand: "<<SF2->getPosition().getTranslation().transpose()<<"\n";
//            std::cout<<"pos EE: "<< model.getSegmentPosition(7).getTranslation().transpose()<<"\n";
//            std::cout<<"ori EE: "<< model.getSegmentPosition(7).getRotation()<<"\n";
        }
        //std::cout<<solver.toString()<<"\n";
        //std::cout<<elapsed_ms/(i+1)<<",";
    }

    std::cout<<std::endl;
    std::cout<<"END OF MAIN\n";
    return 0;
}

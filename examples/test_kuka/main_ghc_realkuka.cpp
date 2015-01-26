
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
#include "orcisir/OrthonormalFamily.h"

#include <stdlib.h>
#include <sys/time.h>

std::pair<VectorXd, MatrixXd> sortRows(const MatrixXd &C, const MatrixXd &J)
{
    int totalTaskDim = J.rows();
    int nDof = J.cols();
    MatrixXd Cii_J = MatrixXd::Zero(totalTaskDim, 1+nDof);

    Cii_J.col(0) = C.diagonal();
    Cii_J.block(0,1,totalTaskDim,nDof) = J;

    int rdim = Cii_J.rows();
    VectorXd tmp;
    for (int rmin=0; rmin<rdim-1;++rmin)
    {
        for(int i=rdim-1; i>rmin; --i)
        {
            if (Cii_J(i,0)>Cii_J(i-1,0))
            {
                tmp = Cii_J.row(i-1);
                Cii_J.row(i-1) = Cii_J.row(i);
                Cii_J.row(i) = tmp;
            }
        }
    }

    return std::pair<VectorXd, MatrixXd>(Cii_J.col(0),Cii_J.block(0,1,totalTaskDim,nDof));
}

void computeProjector(const MatrixXd &C, const MatrixXd &J, MatrixXd& projector)
{
    int totalTaskDim = J.rows();
    int nDof = J.cols();

    VectorXd Cs(nDof);
    MatrixXd Js(totalTaskDim, nDof);


    std::pair<VectorXd, MatrixXd> sortedMatrix = sortRows(C,J);
    Cs = sortedMatrix.first;
    Js = sortedMatrix.second;
    orcisir::OrthonormalFamily onfamily(Js, 1e-9);
    onfamily.computeOrthonormalFamily();
    MatrixXd onb_Js = onfamily.getOnf();
    VectorXi origin = onfamily.getOrigin();

    int k = onfamily.getIndex();
    VectorXd Chat(k);

    for (int j=0; j<k; ++j)
    {
        Chat(j)=Cs(origin(j));
    }


    MatrixXd alpha = MatrixXd(Chat.asDiagonal());
    projector = MatrixXd::Identity(nDof,nDof) - onb_Js.transpose()*alpha*onb_Js;
}

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
    int nDof = model.nbDofs();

    orcisir::OneLevelSolverWithQuadProg   solver;

    orcisir::GHCJTController               ctrl("myCtrl", model, solver, useGSHC, useGrav);


    std::cout<<"INITIALIZE TASKS\n";

//================ FULL STATE ==============================================================================//
    orc::FullModelState   FMS("torqueTask.FModelState", model, orc::FullState::INTERNAL); //INTERNAL, FREE_FLYER
    orc::FullTargetState  FTS("torqueTask.FTargetState", model, orc::FullState::INTERNAL);
    orc::FullStateFeature feat("torqueTask", FMS);
    orc::FullStateFeature featDes("torqueTask.Des", FTS);

    VectorXd qdes(nDof);
    qdes<<0.2,-0.25,0.2,1.3,0.2,-1.2,0.2;
    FTS.set_q(qdes);
//    FTS.set_q(Eigen::VectorXd::Constant(model.nbInternalDofs(), 0.25));

    orcisir::GHCJTTask& accTask = ctrl.createGHCJTTask("accTask", feat, featDes);
//    accTask.initAsAccelerationTask();
    ctrl.addTask(accTask);
    accTask.activateAsObjective();
    accTask.setStiffness(0.05);
    accTask.setDamping(0.02);
    MatrixXd jacobianTask1 = MatrixXd::Zero(feat.getDimension(),nDof);
    jacobianTask1.transposeInPlace();




//================ FRAME ==============================================================================//
    orc::SegmentFrame        SF("frame.SFrame", model, "kuka.07", Eigen::Displacementd());
    orc::TargetFrame         TF("frame.TFrame", model);
    orc::PositionFeature feat2("frame", SF, orc::XYZ);
    orc::PositionFeature featDes2("frame.Des", TF, orc::XYZ);

    TF.setPosition(Eigen::Displacementd(0.3,0.0,0.5));//0.3,0.0,0.5
    TF.setVelocity(Eigen::Twistd());
    TF.setAcceleration(Eigen::Twistd());

    orcisir::GHCJTTask& accTask2 = ctrl.createGHCJTTask("accTask2", feat2, featDes2);
//    accTask2.initAsAccelerationTask();
    ctrl.addTask(accTask2);
    accTask2.activateAsObjective();
    accTask2.setStiffness(16);//30,18 ; 16,12 ; 5,7
    accTask2.setDamping(12);
    MatrixXd jacobianTask2 = MatrixXd::Zero(feat2.getDimension(),nDof);

    ctrl.setActiveTaskVector();
    int nt = ctrl.getNbActiveTask();
    MatrixXd param_priority(nt,nt);
//    param_priority<<0;

//    std::vector< orcisir::ISIRTask* >& activeTask = ctrl.getActiveTask();

    VectorXd qini(7);
    qini<<0.0,-0.05,0.0,1.5,0.0,-1.0,0.0;
    VectorXd q      = Eigen::VectorXd::Constant(model.nbInternalDofs(), 0.2);
    q = qini;
    VectorXd dq     = Eigen::VectorXd::Zero(model.nbInternalDofs());
    VectorXd tau    = Eigen::VectorXd::Zero(model.nbInternalDofs());
    double dt = 0.005;

    model.setJointPositions(q);
    model.setJointVelocities(dq);

    //SIMULATE
    std::cout<<"SIMULATE\n";


    Vector3d error,errDot,errorDot;
    Eigen::Displacementd::Rotation3D Rdes_in_r;
    Matrix3d spaceTransform;
    VectorXd eq(nDof),deq(nDof);



    int t1 = 10000;

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



        // UpdateAugmentedJacobian
        MatrixXd augmentedJacobian=MatrixXd::Zero(feat.getDimension()+feat2.getDimension(), nDof);
        int rowindex = 0;
        int taskdim;

        taskdim = feat.getDimension();
        augmentedJacobian.block(rowindex,0, taskdim, nDof)=accTask.getJacobian();

        rowindex += taskdim;

        taskdim = feat2.getDimension();
        augmentedJacobian.block(rowindex,0, taskdim, nDof)=accTask2.getJacobian();
//        std::cout<<"J2="<<accTask2.getJacobian()<<std::endl;
        Displacementd posEEactual=SF.getPosition();
        Displacementd posEE(0,0,0,posEEactual.getRotation().w(),
                            posEEactual.getRotation().x(),
                            posEEactual.getRotation().y(),
                            posEEactual.getRotation().z());
        Rotation3d qua = posEEactual.getRotation();
        Matrix3d mm;
        mm<<1,2,3,4,5,6,7,8,9;
        std::cout<<"qua="<<mm.adjointTr()<<std::endl;

//        std::cout<<"J2inEE="<<posEE.inverse().adjoint().block(3,3,3,3)*accTask2.getJacobian()<<std::endl;
        //UpdateProjector
        MatrixXd proj(nDof,nDof);
        computeProjector(accTask.getPriority(), augmentedJacobian, proj);
        accTask.setProjector(proj);
        MatrixXd proj2(nDof,nDof);
        computeProjector(accTask2.getPriority(), augmentedJacobian, proj2);
        accTask2.setProjector(proj2);



//        gettimeofday(&start, NULL);
        ctrl.computeOutput(tau);    //compute tau


        eq = FMS.q() - FTS.q();
        deq = FMS.qdot() - FTS.qdot();
        spaceTransform = SF.getPosition().getRotation().adjoint();

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


        tau = (model.getGravityTerms()).tail(model.nbInternalDofs());
//        tau = (model.getGravityTerms()).tail(model.nbInternalDofs())
//                + accTask.getProjector().transpose()*accTask.getJacobian().transpose() * f1
//                + accTask2.getProjector().transpose()*accTask2.getJacobian().transpose() * f2;



        VectorXd ddq = model.getInertiaMatrixInverse() * ( tau - model.getNonLinearTerms() - model.getLinearTerms() - model.getGravityTerms() );
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
//            std::cout<<"pos EE: "<< model.getSegmentPosition(7).getTranslation().transpose()<<"\n";
//            std::cout<<"ori EE: "<< model.getSegmentPosition(7).getRotation()<<"\n";
        }
        //std::cout<<solver.toString()<<"\n";
//        std::cout<<elapsed_ms/(i+1)<<",";
    }

    std::cout<<std::endl;
    std::cout<<"END OF MAIN\n";
    return 0;
}

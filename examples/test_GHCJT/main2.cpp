
#include <iostream>

#include <Eigen/Eigen>

#include "Model3T.h"

#include "orcisir/ISIRController.h"
#include "orcisir/Solvers/OneLevelSolver.h"

#include "orc/control/Feature.h"
#include "orc/control/FullState.h"
#include "orc/control/ControlFrame.h"
#include "orc/control/ControlEnum.h"


int main(int argc, char** argv)
{
    std::cout<<"SET PARAMETERS\n";
    bool useReducedProblem = false;

    orcisir::OneLevelSolverWithQuadProg   internalSolver;
//    orcisir::OneLevelSolverWithQLD        solver;
    
    // INITIALIZE ISIR MODEL, CONTROLLER & TASK MANAGER
    std::cout<<"INITIALIZE ISIR MODEL, CONTROLLER\n";
    Model3T                         model("Model3T");
    orcisir::ISIRController         ctrl("myCtrl", model, internalSolver, useReducedProblem);


    //CREATE SOME TASKS
    std::cout<<"CREATE SOME TASKS\n";
    orc::FullModelState   FMS("fullTask.FModelState" , model, orc::FullState::INTERNAL); //INTERNAL, FREE_FLYER
    orc::FullTargetState  FTS("fullTask.FTargetState", model, orc::FullState::INTERNAL);
    orc::FullStateFeature feat("fullTask.feat", FMS);
    orc::FullStateFeature featDes("fullTask.featDes", FTS);

    FTS.set_q(Eigen::VectorXd::Zero(model.nbInternalDofs()));

    orcisir::ISIRTask& accTask = ctrl.createISIRTask("fullTask", feat, featDes);
    accTask.initAsAccelerationTask();
    ctrl.addTask(accTask);
    accTask.activateAsObjective();
    accTask.setStiffness(9);
    accTask.setDamping(6);
    accTask.setWeight(0.000001);
//    MatrixXd  alpha_posture = MatrixXd::Zero(model.nbInternalDofs(),model.nbInternalDofs());
//    accTask.setPriority(alpha_posture);
//    std::cout<<accTask.getPriority()<<"\n";


    orc::SegmentFrame        SF("frameTask.SFrame", model, "Model3T.segment_3", Eigen::Displacementd());
    orc::TargetFrame         TF("frameTask.TFrame", model);
    orc::PositionFeature feat2("frameTask.feat", SF, orc::XYZ);
    orc::PositionFeature featDes2("frameTask.featDes", TF, orc::XYZ);

    TF.setPosition(Eigen::Displacementd(0.2,0.3,0.4));
    TF.setVelocity(Eigen::Twistd());
    TF.setAcceleration(Eigen::Twistd());

    orcisir::ISIRTask& accTask2 = ctrl.createISIRTask("frameTask", feat2, featDes2);
    accTask2.initAsAccelerationTask();
    ctrl.addTask(accTask2);
    accTask2.activateAsObjective();
    accTask2.setStiffness(9);
    accTask2.setDamping(6);
    
    

    VectorXd q      = Eigen::VectorXd::Constant(model.nbInternalDofs(), 0.1);
    VectorXd dq     = Eigen::VectorXd::Zero(model.nbInternalDofs());
    VectorXd tau    = Eigen::VectorXd::Zero(model.nbInternalDofs());
    double dt = 0.01;

    //SIMULATE
    std::cout<<"SIMULATE\n";
    for (int i=0; i<1000; i++)
    {
        std::cout<<"- -  --- - - - -- - - -- - "<<i<<"\n";
        ctrl.computeOutput(tau);    //compute tau
//        std::cout<<"tau: "<<tau.transpose()<<"\n";

        ctrl.computeOutput(tau);    //compute tau
        //VectorXd ddq = model.getAccelerationVariable().getValue();
        //VectorXd ddq = model.getInertiaMatrixInverse() * ( tau - model.getNonLinearTerms() - model.getLinearTerms() - model.getGravityTerms() );
        VectorXd ddq = model.getInertiaMatrixInverse() * ( tau - model.getNonLinearTerms() - model.getLinearTerms() - model.getGravityTerms());

        dq += ddq * dt;
        q  += dq  * dt;
//        std::cout<<"ddq: "<<ddq.transpose()<<"\n";

        dq += ddq * dt;
        q  += dq  * dt;

        model.setJointPositions(q);
        model.setJointVelocities(dq);

//        std::cout<<"pos seg3: "<< model.getSegmentPosition(3).getTranslation().transpose()<<"\n";
    }



    return 0;
}

//#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
//#include <Eigen/Sparse>
//#include <vector>
//typedef Eigen::SparseMatrix<double> SpMat;

//int main()
//{
////int rows = 6;
////int cols = 6;
////SpMat mat(rows,cols);


////mat.insert(1,2) = 20; // alternative: mat.coeffRef(i,j) += v_ij;
////mat.insert(1,3) = 30;
////mat.coeffRef(1,2) = -20;
////std::cout<<mat<<"\n";
////std::cout<<mat.outerSize()<<"\n";
////for (int k=0; k<mat.outerSize(); ++k)
////{
////    for (SparseMatrix<double>::InnerIterator it(mat,k); it; ++it)
////    {
////        std::cout<<it.value()<<"\n";
////        std::cout<<it.row()<<"\n"; // row index
////        std::cout<<it.col()<<"\n"; // col index (here it is equal to k)
////        std::cout<<it.index()<<"\n"; // inner index, here it is equal to it.row()
////    }
////}


////Matrix<double, 6, 6> m = Matrix<double, 6, 6>::Zero();
////std::cout<<(Matrix<double, 6, 6>)mat+m<<"\n";

////Matrix< double,Dynamic, Dynamic >  alpha;
////alpha = Matrix<double,3,3>::Zero();

////alpha(1,2) = 0.5;
////std::cout<<alpha<<"\n";


//MatrixXd  alpha;
//alpha = Matrix3d::Zero();

//alpha(1,2) = 0.5;
//std::cout<<alpha<<"\n";

//return 0;
//}





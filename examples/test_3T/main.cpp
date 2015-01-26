
#include <iostream>
#include <map>

#include <Eigen/Eigen>

#include "Model3T.h"
#include "orcisir/OrthonormalFamily.h"

#include "orcisir/ISIRController.h"
#include "orcisir/Solvers/OneLevelSolver.h"
#include "orc/control/Feature.h"
#include "orc/control/FullState.h"
#include "orc/control/ControlFrame.h"
#include "orc/control/ControlEnum.h"
#include "orcisir/Constraints/ISIRConstraint.h"
#include "orcisir/Constraints/GSHCAccelerationConstraint.h"

//void Sort(MatrixXd &m)
//{

//    int rdim = m.rows();
//    VectorXd tmp;
//    for(int i=0; i<rdim-1; ++i)
//    {
//        if (m(i,0)>m(i+1,0))
//        {
//            tmp = m.row(i+1);
//            m.row(i+1) = m.row(i);
//            m.row(i) = tmp;

//        }
//    }
//}


//#include <time.h>

//class vectorMatrixPair {
//public:
//    vectorMatrixPair(double d, Eigen::MatrixXd v):first(d),second(v){}
//    double first;
//    Eigen::MatrixXd second;
//    bool operator < (const vectorMatrixPair &pair)const
//    {
//        return pair.first < first;
//    }
//};



//std::pair<VectorXd, MatrixXd> sortRows(const MatrixXd &C, const MatrixXd &J)
//{
//    int totalTaskDim = J.rows();
//    int nDof = J.cols();

//    std::vector<vectorMatrixPair> vect;
//    for (int i = 0; i<totalTaskDim; ++i)
//    {
//        vectorMatrixPair pair(C(i,i),J.row(i));
//        vect.push_back(pair);
//    }

//    sort(vect.begin(), vect.end());
//    MatrixXd Cii_J = MatrixXd::Zero(totalTaskDim, 1+nDof);
//    for (int i = 0; i<totalTaskDim; ++i)
//    {
//        Cii_J(i,0) = vect[i].first;
//        Cii_J.block(i,1,1,nDof) = vect[i].second;
//    }
////    Cii_J.col(0) = C.diagonal();
////    Cii_J.block(0,1,totalTaskDim,nDof) = J;
////    VectorXd col = Cii_J.col(0);
////    std::sort(index_begin<double>(Cii_J), index_end  <double>(Cii_J));

//    return std::pair<VectorXd, MatrixXd>(Cii_J.col(0),Cii_J.block(0,1,totalTaskDim,nDof));
//}

//std::pair<VectorXd, MatrixXd> sortRows2(const MatrixXd &C, const MatrixXd &J)
//{
//    int totalTaskDim = J.rows();
//    int nDof = J.cols();
//    MatrixXd Cii_J = MatrixXd::Zero(totalTaskDim, 1+nDof);

//    Cii_J.col(0) = C.diagonal();
//    Cii_J.block(0,1,totalTaskDim,nDof) = J;

//    int rdim = Cii_J.rows();
//    VectorXd tmp;
//    for (int rmin=0; rmin<rdim-1;++rmin)
//    {
//        for(int i=rdim-1; i>rmin; --i)
//        {
//            if (Cii_J(i,0)>Cii_J(i-1,0))
//            {
//                tmp = Cii_J.row(i-1);
//                Cii_J.row(i-1) = Cii_J.row(i);
//                Cii_J.row(i) = tmp;
//            }
//        }
//    }

//    return std::pair<VectorXd, MatrixXd>(Cii_J.col(0),Cii_J.block(0,1,totalTaskDim,nDof));
//}

//int main()
//{
//    MatrixXd C(3,3), C2(3,3);
//    C<<3,0,0,0,1,0,0,0,2;
//    C2<<3,0,0,0,1,0,0,0,2;
//    MatrixXd m(3,2),m2(3,2);
//    m<< 2,3,5,6,8,9;
//    m2<< 2,3,5,6,8,9;
////    std::cout<<C<<std::endl;
////    std::cout<<m<<std::endl;

//    clock_t t;

////    std::cout<<"sorted matrix"<<std::endl;
////    t = clock();
////    std::pair<VectorXd, MatrixXd> result = sortRows(C,m);
////    for(int i=0; i<10000;++i)
////    {sortRows(C,m);}
////    t = clock()-t;
////    std::cout<<result.first<<std::endl;
////    std::cout<<result.second<<std::endl;
////    std::cout<<((float)t)/CLOCKS_PER_SEC<<"s"<<std::endl;

////    std::cout<<"sorted matrix 2"<<std::endl;
////    t = clock();
////    std::pair<VectorXd, MatrixXd> result2 = sortRows2(C2,m2);
////    for(int i=0; i<10000;++i)
////    {sortRows2(C2,m2);}
////    t = clock()-t;
////    std::cout<<result2.first<<std::endl;
////    std::cout<<result2.second<<std::endl;
////    std::cout<<((float)t)/CLOCKS_PER_SEC<<"s"<<std::endl;

////    OrthonormalFamily onfamily(m, 1e-9);
////    onfamily.computeOrthonormalFamily();
////    std::cout<<"index"<<std::endl;
////    std::cout<<onfamily.getIndex()<<std::endl;
////    std::cout<<"onf"<<std::endl;
////    std::cout<<onfamily.getOnf()<<std::endl;
////    std::cout<<"origin"<<std::endl;
////    std::cout<<onfamily.getOrigin()<<std::endl;

//    VectorXd Chat=C.diagonal();
//    t = clock();

//    for(int i=0; i<1000000;++i)
//    {C*m;}
//    t=clock()-t;
//    std::cout<<"result1"<<std::endl;
//    m =C*m;
//    std::cout<<m<<std::endl;

//    std::cout<<((float)t)/CLOCKS_PER_SEC<<"s"<<std::endl;

//    t=clock();
//    for(int i=0; i<1000000;++i)
//    {
//        for(int j=0;j<C.rows();++j)
//        {
//            Chat(j)*m.row(j);
//        }
//    }
//    t=clock()-t;
//    std::cout<<"result2"<<std::endl;
//    for(int j=0;j<C.rows();++j)
//    {
//        m2.row(j)=Chat(j)*m2.row(j);
//    }

//    std::cout<<m2<<std::endl;
//    std::cout<<((float)t)/CLOCKS_PER_SEC<<"s"<<std::endl;
//    return 0;
//}

int main(int argc, char** argv)
{
    std::cout<<"SET PARAMETERS\n";
    bool useReducedProblem = false;
    bool useGSHC = true;

    orcisir::OneLevelSolverWithQuadProg   internalSolver;
//    orcisir::OneLevelSolverWithQLD        solver;
    
    // INITIALIZE ISIR MODEL, CONTROLLER & TASK MANAGER
    std::cout<<"INITIALIZE ISIR MODEL, CONTROLLER\n";
    Model3T                         model("Model3T");
    orcisir::ISIRController         ctrl("myCtrl", model, internalSolver, useReducedProblem, useGSHC);


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

    orc::SegmentFrame        SF("frameTask.SFrame", model, "Model3T.segment_3", Eigen::Displacementd());
    orc::TargetFrame         TF("frameTask.TFrame", model);
    orc::PositionFeature feat2("frameTask.feat", SF, orc::XYZ);
    orc::PositionFeature featDes2("frameTask.featDes", TF, orc::XYZ);

    TF.setPosition(Eigen::Displacementd(0.2,0.3,0.4));
    TF.setVelocity(Eigen::Twistd());
    TF.setAcceleration(Eigen::Twistd());
//    TF.setWrench(Eigen::Wrenchd(0,0,0,0,2,0));
    Eigen::Vector3d pos,posdes, eff;
//    TF.setWrench(Eigen::Wrenchd(0,0,0,0,0,0));


    orcisir::ISIRTask& accTask2 = ctrl.createISIRTask("frameTask", feat2, featDes2);
//    accTask2.initAsAccelerationTask();
    accTask2.initAsForceTask();
    ctrl.addTask(accTask2);
    accTask2.activateAsObjective();
    accTask2.setStiffness(9);
    accTask2.setDamping(6);
    accTask2.setWeight(1);

    posdes<<0.2,0.3,0.4;
    double kp = accTask2.getStiffness()(0,0);
    double kd = accTask2.getDamping()(0,0);



        ctrl.setActiveTaskVector();

        // SET TASK PRIORITIES, COMPUTE TASK PROJECTORS
        int nt = ctrl.getNbActiveTask();

        MatrixXd param_priority(nt,nt);
//        std::vector< orcisir::ISIRTask* >& activeTask = ctrl.getActiveTask();

        param_priority<<0,1,0,0;
        ctrl.setTaskProjectors(param_priority);

    //    orc::EqualZeroConstraintPtr< orcisir::GSHCFunction >      gshcFunction;
    //    gshcFunction = new orcisir::GSHCFunction(model, activeTask, ctrl.getTaskVariables());
//        ctrl.addConstraint(ctrl.gshcFunction.getConstraint());
        ctrl.setGSHCConstraint();



    //SIMULATE
    VectorXd q      = Eigen::VectorXd::Constant(model.nbInternalDofs(), 0.1);
    VectorXd dq     = Eigen::VectorXd::Zero(model.nbInternalDofs());
    VectorXd tau    = Eigen::VectorXd::Zero(model.nbInternalDofs());
    VectorXd ddq;
    double dt = 0.01;

    std::cout<<"SIMULATE\n";
    for (int i=0; i<1000; i++)
    {
        pos = model.getSegmentPosition(3).getTranslation();
        eff = kp*(posdes-pos)-kd*model.getSegmentVelocity(3).getLinearVelocity();
        TF.setWrench(Eigen::Wrenchd(0,0,0,eff(0),eff(1),eff(2)));
//        std::cout<<"ddq: "<<model.getSegmentVelocity(3).getLinearVelocity().transpose()<<"\n";
        if (i==600)
        {
            param_priority(0,1)=0;
            param_priority(1,0)=1;
//            std::cout<<SF.getWrench().getForce()<<std::endl;
//            std::cout<<TF.getWrench().getForce()<<std::endl;
//            std::cout<<"change priority"<<std::endl;
        }
        ctrl.setTaskProjectors(param_priority);
        ctrl.doUpdateProjector();
        ctrl.computeOutput(tau);    //compute tau
//        std::cout<<"tau: "<<tau.transpose()<<"\n";

        //VectorXd ddq = model.getAccelerationVariable().getValue();
        //VectorXd ddq = model.getInertiaMatrixInverse() * ( tau - model.getNonLinearTerms() - model.getLinearTerms() - model.getGravityTerms() );
        ddq = model.getInertiaMatrixInverse() * ( tau - model.getNonLinearTerms() - model.getLinearTerms() - model.getGravityTerms());

        dq += ddq * dt;
        q  += dq  * dt;



        model.setJointPositions(q);
        model.setJointVelocities(dq);
        if (i==599||i==999)
        {
            std::cout<<"- -  --- - - - -- - - -- - "<<i<<"\n";

            std::cout<<"ddq: "<<ddq.transpose()<<"\n";
            std::cout<<"pos seg3: "<< model.getSegmentPosition(3).getTranslation().transpose()<<"\n";

        }

    }
//    std::cout<<accTask.getTaskAccelerationVariable().getValue()<<std::endl;
//    std::cout<<accTask2.getTaskAccelerationVariable().getValue()<<std::endl;
//    std::cout<<model.getAccelerationVariable().getValue()<<std::endl;

    return 0;
}


//int main(int argc, char** argv)
//{
//    std::cout<<"SET PARAMETERS\n";
//    bool useReducedProblem = false;
//    bool useGSHC = true;

//    orcisir::OneLevelSolverWithQuadProg   internalSolver;
////    orcisir::OneLevelSolverWithQLD        solver;

//    // INITIALIZE ISIR MODEL, CONTROLLER & TASK MANAGER
//    std::cout<<"INITIALIZE ISIR MODEL, CONTROLLER\n";
//    Model3T                         model("Model3T");
//    orcisir::ISIRController         ctrl("myCtrl", model, internalSolver, useReducedProblem, useGSHC);


//    //CREATE SOME TASKS
//    std::cout<<"CREATE SOME TASKS\n";
//    orc::FullModelState   FMS("fullTask.FModelState" , model, orc::FullState::INTERNAL); //INTERNAL, FREE_FLYER
//    orc::FullTargetState  FTS("fullTask.FTargetState", model, orc::FullState::INTERNAL);
//    orc::FullStateFeature feat("fullTask.feat", FMS);
//    orc::FullStateFeature featDes("fullTask.featDes", FTS);
//    FTS.set_q(Eigen::VectorXd::Zero(model.nbInternalDofs()));

//    orcisir::ISIRTask& accTask = ctrl.createISIRTask("fullTask", feat, featDes);

//    accTask.initAsAccelerationTask();
//    ctrl.addTask(accTask);
//    accTask.activateAsObjective();
//    accTask.setStiffness(9);
//    accTask.setDamping(6);
//    accTask.setWeight(0.000001);

//    orc::SegmentFrame        SF("frameTask.SFrame", model, "Model3T.segment_3", Eigen::Displacementd());
//    orc::TargetFrame         TF("frameTask.TFrame", model);
//    orc::PositionFeature feat2("frameTask.feat", SF, orc::XYZ);
//    orc::PositionFeature featDes2("frameTask.featDes", TF, orc::XYZ);

//    TF.setPosition(Eigen::Displacementd(0.2,0.3,0.4));
//    TF.setVelocity(Eigen::Twistd());
//    TF.setAcceleration(Eigen::Twistd());
////    TF.setWrench(Eigen::Wrenchd(0,0,0,0,2,0));
//    Eigen::Vector3d pos,posdes, eff;
////    TF.setWrench(Eigen::Wrenchd(0,0,0,0,0,0));


//    orcisir::ISIRTask& accTask2 = ctrl.createISIRTask("frameTask", feat2, featDes2);
//    accTask2.initAsAccelerationTask();
////    accTask2.initAsForceTask();
//    ctrl.addTask(accTask2);
//    accTask2.activateAsObjective();
//    accTask2.setStiffness(9);
//    accTask2.setDamping(6);
//    accTask2.setWeight(1);

//    posdes<<0.2,0.3,0.4;
//    double kp = accTask2.getStiffness()(0,0);
//    double kd = accTask2.getDamping()(0,0);



//        ctrl.setActiveTaskVector();

//        // SET TASK PRIORITIES, COMPUTE TASK PROJECTORS
//        int nt = ctrl.getNbActiveTask();

//        MatrixXd param_priority(nt,nt);
////        std::vector< orcisir::ISIRTask* >& activeTask = ctrl.getActiveTask();

//        param_priority<<0,1,0,0;
//        ctrl.setTaskProjectors(param_priority);

//    //    orc::EqualZeroConstraintPtr< orcisir::GSHCFunction >      gshcFunction;
//    //    gshcFunction = new orcisir::GSHCFunction(model, activeTask, ctrl.getTaskVariables());
////        ctrl.addConstraint(ctrl.gshcFunction.getConstraint());
//        ctrl.setGSHCConstraint();



//    //SIMULATE
//    VectorXd q      = Eigen::VectorXd::Constant(model.nbInternalDofs(), 0.1);
//    VectorXd dq     = Eigen::VectorXd::Zero(model.nbInternalDofs());
//    VectorXd tau    = Eigen::VectorXd::Zero(model.nbInternalDofs());
//    VectorXd ddq;
//    double dt = 0.01;

//    std::cout<<"SIMULATE\n";
//    for (int i=0; i<1000; i++)
//    {
////        pos = model.getSegmentPosition(3).getTranslation();
////        eff = kp*(posdes-pos)-kd*model.getSegmentVelocity(3).getLinearVelocity();
////        TF.setWrench(Eigen::Wrenchd(0,0,0,eff(0),eff(1),eff(2)));
////        std::cout<<"ddq: "<<model.getSegmentVelocity(3).getLinearVelocity().transpose()<<"\n";
//        if (i==600)
//        {
//            param_priority(0,1)=0;
//            param_priority(1,0)=1;
////            std::cout<<SF.getWrench().getForce()<<std::endl;
////            std::cout<<TF.getWrench().getForce()<<std::endl;
////            std::cout<<"change priority"<<std::endl;
//        }
//        ctrl.setTaskProjectors(param_priority);
//        ctrl.doUpdateProjector();
//        ctrl.computeOutput(tau);    //compute tau
////        std::cout<<"tau: "<<tau.transpose()<<"\n";

//        //VectorXd ddq = model.getAccelerationVariable().getValue();
//        //VectorXd ddq = model.getInertiaMatrixInverse() * ( tau - model.getNonLinearTerms() - model.getLinearTerms() - model.getGravityTerms() );
//        ddq = model.getInertiaMatrixInverse() * ( tau - model.getNonLinearTerms() - model.getLinearTerms() - model.getGravityTerms());

//        dq += ddq * dt;
//        q  += dq  * dt;



//        model.setJointPositions(q);
//        model.setJointVelocities(dq);
//        if (i==599||i==999)
//        {
//            std::cout<<"- -  --- - - - -- - - -- - "<<i<<"\n";

//            std::cout<<"ddq: "<<ddq.transpose()<<"\n";
//            std::cout<<"pos seg3: "<< model.getSegmentPosition(3).getTranslation().transpose()<<"\n";

//        }

//    }
////    std::cout<<accTask.getTaskAccelerationVariable().getValue()<<std::endl;
////    std::cout<<accTask2.getTaskAccelerationVariable().getValue()<<std::endl;
////    std::cout<<model.getAccelerationVariable().getValue()<<std::endl;

//    return 0;
//}

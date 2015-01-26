
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




//int main()
//{
//    MatrixXd m(3,3);
//    m<< 3,2,3,1,5,6,2,8,9;
//    std::cout<<m<<std::endl;

//    Sort(m);
//    std::cout<<"sorted matrix"<<std::endl;
//    std::cout<<m<<std::endl;

//    OrthonormalFamily onfamily(m, 1e-9);
//    onfamily.computeOrthonormalFamily();
//    std::cout<<"index"<<std::endl;
//    std::cout<<onfamily.getIndex()<<std::endl;
//    std::cout<<"onf"<<std::endl;
//    std::cout<<onfamily.getOnf()<<std::endl;
//    std::cout<<"origin"<<std::endl;
//    std::cout<<onfamily.getOrigin()<<std::endl;

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
//    accTask.setWeight(0.000001);
    accTask.setWeight(1);

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
    accTask2.setWeight(1);


    ctrl.setActiveTaskVector();



    // SET TASK PRIORITIES, COMPUTE TASK PROJECTORS


    enum TASKNAME { FULLTASK, FRAMETASK };
    std::map<std::string,int> task_name_number_mapping;
    std::map<std::string,int>::iterator it_name_number = task_name_number_mapping.begin();
    task_name_number_mapping.insert (it_name_number, std::pair<std::string,int>("fullTask",FULLTASK));
    task_name_number_mapping.insert (it_name_number, std::pair<std::string,int>("frameTask",FRAMETASK));


    int nt = ctrl.getNbActiveTask();
    MatrixXd param_priority(nt,nt);
    std::vector< orcisir::ISIRTask* >& activeTask = ctrl.getActiveTask();
    param_priority<<0,1,0,0;

//    if(useGSHC)
//    {
    for (int i=0; i<nt; ++i)
    {

        switch (task_name_number_mapping.find(activeTask[i]->getTaskName())->second)
        {
            case(FULLTASK):
            {
                for (int j=0; j<nt; ++j)
                {
                        switch(task_name_number_mapping.find(activeTask[j]->getTaskName())->second)
                        {
                            case(FULLTASK):
                            {
                                param_priority(i,j) = 0;
                                break;
                            }
                            case(FRAMETASK):
                            {
                                param_priority(i,j) = 1;
                                break;
                            }
                            default:
                            {
                                throw std::runtime_error(std::string("Priority setting: Unhandle case of task name."));
                                break;
                            }
                        }
                }

                break;
            }

            case(FRAMETASK):
            {
                for (int j=0; j<nt; ++j)
                {
                        switch(task_name_number_mapping.find(activeTask[j]->getTaskName())->second)
                        {
                            case(FULLTASK):
                            {
                                param_priority(i,j) = 0;
                                break;
                            }
                            case(FRAMETASK):
                            {
                                param_priority(i,j) = 0;
                                break;
                            }
                            default:
                            {
                                throw std::runtime_error(std::string("Priority setting: Unhandle case of task name."));
                                break;
                            }
                        }
                }

                break;
            }
        }

    }


    ctrl.setTaskProjectors(param_priority);


    for (int i=0; i<activeTask.size(); ++i)
    {
        std::cout<<"jacobian of "<<activeTask[i]->getTaskName()<<" task:"<<std::endl;
        std::cout<<activeTask[i]->getJacobian()<<std::endl;
        std::cout<<"priority of "<<activeTask[i]->getTaskName()<<" task:"<<std::endl;
        std::cout<<activeTask[i]->getPriority()<<std::endl;
        std::cout<<"projector of "<<activeTask[i]->getTaskName()<<" task:"<<std::endl;
        std::cout<<activeTask[i]->getProjector()<<std::endl;
    }


    CompositeVariable var("taskVariables", model.getAccelerationVariable());
    for (int i=0; i<activeTask.size(); ++i)
    {
        var.add(activeTask[i]->getTaskAccelerationVariable());
    }
    orc::EqualZeroConstraintPtr< orcisir::GSHCFunction >      gshcFunction;
    gshcFunction = new orcisir::GSHCFunction(model, activeTask, var);


    ctrl.addConstraint(gshcFunction.getConstraint());
//    }

    //SIMULATE
    VectorXd q      = Eigen::VectorXd::Constant(model.nbInternalDofs(), 0.1);
    VectorXd dq     = Eigen::VectorXd::Zero(model.nbInternalDofs());
    VectorXd tau    = Eigen::VectorXd::Zero(model.nbInternalDofs());
    double dt = 0.01;

    std::cout<<"SIMULATE\n";
    for (int i=0; i<1000; i++)
    {
        std::cout<<"- -  --- - - - -- - - -- - "<<i<<"\n";
        ctrl.computeOutput(tau);    //compute tau
//        std::cout<<"tau: "<<tau.transpose()<<"\n";

        //VectorXd ddq = model.getAccelerationVariable().getValue();
        //VectorXd ddq = model.getInertiaMatrixInverse() * ( tau - model.getNonLinearTerms() - model.getLinearTerms() - model.getGravityTerms() );
        VectorXd ddq = model.getInertiaMatrixInverse() * ( tau - model.getNonLinearTerms() - model.getLinearTerms() - model.getGravityTerms());

        dq += ddq * dt;
        q  += dq  * dt;
        std::cout<<"ddq: "<<ddq.transpose()<<"\n";


        model.setJointPositions(q);
        model.setJointVelocities(dq);

        std::cout<<"pos seg3: "<< model.getSegmentPosition(3).getTranslation().transpose()<<"\n";
    }


    return 0;
}

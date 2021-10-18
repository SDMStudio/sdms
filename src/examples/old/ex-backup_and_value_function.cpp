#include <sdm/core/space/discrete_space.hpp>

#include <sdm/world/mdp.hpp>
#include <sdm/world/solvable_by_mdp.hpp>
#include <sdm/world/belief_mdp.hpp>

#include <sdm/parser/parser.hpp>

#include <sdm/utils/value_function/vfunction/tabular_value_function.hpp>
#include <sdm/utils/value_function/vfunction/point_set_value_function.hpp>
#include <sdm/utils/value_function/vfunction/pwlc_value_function.hpp>

#include <sdm/utils/value_function/backup/tabular_backup.hpp>
#include <sdm/utils/value_function/backup/maxplan_backup.hpp>

using namespace sdm;


int main(int argc, char **argv)
{
    auto mdp_tiger = sdm::parser::parse_file("../data/world/dpomdp/tiger.dpomdp");

    auto state_space = std::static_pointer_cast<DiscreteSpace>(mdp_tiger->getStateSpace()); //std::make_shared<DiscreteSpace>(std::vector<std::shared_ptr<Item>>{state_0, state_1, state_2, state_3});
    auto action_space = mdp_tiger->getActionSpace(); //= std::make_shared<MultiDiscreteSpace>(std::vector<std::shared_ptr<Space>>{single_action_space, single_action_space});
    auto rew = mdp_tiger->getRewardSpace(); 
    auto dynamics = mdp_tiger->getStateDynamics();

    auto start_distrib = mdp_tiger->getStartDistribution();
    auto obs_space = mdp_tiger->getObservationSpace(0);
    auto obs_dynamics = mdp_tiger->getObservationDynamics();

    number horizon = 5;

    // Creation of the MMDP
    auto mdp = std::make_shared<MDP>(state_space, action_space, rew, dynamics,start_distrib,horizon,1.);

    // Creation of HSVI problem and Resolution 
    std::shared_ptr<SolvableByHSVI> hsvi_mdp = std::make_shared<SolvableByMDP>(mdp);

    // horizon = horizon * mdp->getNumAgents();
    auto tabular_backup = std::make_shared<TabularBackup>(hsvi_mdp);

    auto ub = std::make_shared<TabularValueFunction>(horizon,1000,tabular_backup);

    auto state = state_space->sample()->toState();

    std::cout<<" ******* Tabular Value Function ******"<<std::endl;

    std::cout<<"Get Support "<<ub->getSupport(0)<<std::endl;
    std::cout<<"Get isFinite "<<ub->isFiniteHorizon()<<std::endl;

    std::cout<<"Max At before update "<<tabular_backup->getMaxAt(ub,state,0)<<std::endl;
    std::cout<<"Backup "<<tabular_backup->backup(ub,state,0)<<std::endl;
    ub->updateValueAt(state);
    std::cout<<"Max At after update "<<tabular_backup->getMaxAt(ub,state,0)<<std::endl;
    std::cout<<"Get Support "<<ub->getSupport(0)<<std::endl;

    std::cout<<"Best Action "<<tabular_backup->getBestAction(ub,state,0)->str()<<std::endl;


    std::cout<<" ******* Point Value Function ******"<<std::endl;

    ub = std::make_shared<PointSetValueFunction>(horizon,1000,tabular_backup);
    auto lb = std::make_shared<PointSetValueFunction>(horizon,-1000,tabular_backup);

    auto algorithm = std::make_shared<HSVI>(hsvi_mdp, lb, ub, mdp->getHorizon(), 0.01);
    algorithm->initialize();
    algorithm->solve();

    // Test Hyperplan ! 

    // Creation of the MMDP
    auto pomdp = std::make_shared<POMDP>(state_space, action_space,obs_space, rew, dynamics,obs_dynamics,start_distrib,horizon,1.);

    // Creation of HSVI problem and Resolution 
    std::shared_ptr<SolvableByHSVI> hsvi_mdp = std::make_shared<BeliefMDP>(pomdp);

    // horizon = horizon * mdp->getNumAgents();
    auto maxplan_backup = std::make_shared<MaxPlanBackup>(hsvi_mdp);
    auto init_lb = std::make_shared<MinInitializer>(hsvi_mdp);

    auto lb = std::make_shared<PWLCValueFunction>(horizon,init_lb,maxplan_backup);
    lb->initialize();

    std::cout<<lb->str()<<std::endl;

    auto state = hsvi_mdp->getInitialState();

    std::cout<<"state : "<<state->str()<<std::endl;

    std::cout<<" ******* Hyerplan Value Function ******"<<std::endl;

    std::cout<<"Get Support size"<<lb->getSupport(0).size()<<std::endl;
    std::cout<<"Get isFinite "<<lb->isFiniteHorizon()<<std::endl;

    std::cout<<"Max At before update, value :"<<maxplan_backup->getMaxAt(lb,state,0).first<<", state : "<<maxplan_backup->getMaxAt(lb,state,0).second->str()<<std::endl;
    std::cout<<"Backup "<<maxplan_backup->backup(lb,state,0)->str()<<std::endl;
    lb->updateValueAt(state);
    std::cout<<"Max At before update, value :"<<maxplan_backup->getMaxAt(lb,state,0).first<<", state : "<<maxplan_backup->getMaxAt(lb,state,0).second->str()<<std::endl;
    std::cout<<"Get Support size"<<lb->getSupport(0).size()<<std::endl;

    std::cout<<lb->str()<<std::endl;


    std::cout<<"Best Action "<<maxplan_backup->getGreedyAction(lb,state,0)->str()<<std::endl;



} // END main

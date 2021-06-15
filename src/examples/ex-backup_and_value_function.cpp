#include <sdm/core/space/discrete_space.hpp>

#include <sdm/world/mdp.hpp>
#include <sdm/world/solvable_by_mdp.hpp>

#include <sdm/parser/parser.hpp>

#include <sdm/utils/value_function/tabular_value_function.hpp>
#include <sdm/utils/value_function/point_set_value_function.hpp>

using namespace sdm;


int main(int argc, char **argv)
{
    auto mdp_tiger = sdm::parser::parse_file("../data/world/dpomdp/tiger.dpomdp");

    auto state_space = std::static_pointer_cast<DiscreteSpace>(mdp_tiger->getStateSpace()); //std::make_shared<DiscreteSpace>(std::vector<std::shared_ptr<Item>>{state_0, state_1, state_2, state_3});
    auto action_space = mdp_tiger->getActionSpace(); //= std::make_shared<MultiDiscreteSpace>(std::vector<std::shared_ptr<Space>>{single_action_space, single_action_space});
    auto rew = mdp_tiger->getReward(); 
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
    algorithm->do_initialize();
    algorithm->do_solve();


    // std::cout<<"state dominate a balue of 10000  "<<ub->is_dominated(state,10000,0)


} // END main

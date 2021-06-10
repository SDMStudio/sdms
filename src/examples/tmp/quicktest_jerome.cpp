#include <iostream>

#include <memory>
#include <sdm/exception.hpp>

// #include <sdm/algorithms/hsvi.hpp>

#include <sdm/core/action/action.hpp>
#include <sdm/core/action/base_action.hpp>
#include <sdm/core/state/state.hpp>
#include <sdm/core/state/base_state.hpp>
#include <sdm/core/state/serialized_state.hpp>
#include <sdm/core/base_observation.hpp>

#include <sdm/core/space/discrete_space.hpp>
#include <sdm/core/space/multi_discrete_space.hpp>
#include <sdm/core/distribution.hpp>
#include <sdm/core/reward/tabular_reward.hpp>
#include <sdm/core/dynamics/tabular_state_dynamics.hpp>
#include <sdm/core/dynamics/tabular_observation_dynamics.hpp>

#include <sdm/world/mdp.hpp>
#include <sdm/world/mmdp.hpp>
#include <sdm/world/solvable_by_mdp.hpp>
#include <sdm/world/serialized_mmdp.hpp>
#include <sdm/world/mpomdp.hpp>
#include <sdm/world/pomdp.hpp>
// #include <sdm/world/belief_mdp.hpp>

#include <sdm/parser/parser.hpp>

#include <sdm/utils/value_function/point_set_value_function.hpp>
#include <sdm/utils/value_function/tabular_backup.hpp>

#include <sdm/utils/value_function/initializer.hpp>
#include <sdm/utils/value_function/initializer/mdp_initializer.hpp>

using namespace sdm;

int main(int argc, char **argv)
{
    auto mdp_tiger = sdm::parser::parse_file("../data/world/dpomdp/tiger.dpomdp");
    auto state_space = mdp_tiger->getStateSpace(); //std::make_shared<DiscreteSpace>(std::vector<std::shared_ptr<Item>>{state_0, state_1, state_2, state_3});
    auto action_space = mdp_tiger->getActionSpace(); //= std::make_shared<MultiDiscreteSpace>(std::vector<std::shared_ptr<Space>>{single_action_space, single_action_space});
    auto rew = mdp_tiger->getReward(); 
    auto dynamics = mdp_tiger->getStateDynamics();

    auto start_distrib = mdp_tiger->getStartDistribution();


    number horizon = 5;

    //Creation of the MMDP
    auto mdp = std::make_shared<MDP>(state_space, action_space, rew, dynamics ,start_distrib,horizon,1.);

    // Creation of the Serial MMDP with the MMDP
    // auto serial_mmdp = std::make_shared<SerializedMMDP>(mdp);

    // Some function of the Serial MMDP 
    // std::cout << "#> NumAgents = " << pomdp->getNumAgents() << std::endl;
    // std::cout << "#> Discount = " << pomdp->getDiscount() << std::endl;
    // // std::cout << "#> isLastAgent(0)"<<pomdp->isLastAgent(0) << std::endl;
    // // std::cout << "#> getAgentId( t = 3)"<<pomdp->getAgentId(3) << std::endl;
    // std::cout << "#> Initial Distribution "<<pomdp->getStartDistribution() << std::endl;


    // for(const auto &state_tmp : *pomdp->getStateSpace(0))
    // {
    //     std::shared_ptr<State> state = std::static_pointer_cast<State>(state_tmp);

    //     for(auto action_tmp : *pomdp->getActionSpace(0))
    //     {
    //         std::shared_ptr<Action> action = std::static_pointer_cast<Action>(action_tmp);

    //         for(const auto &next_state_tmp : pomdp->getReachableStates(state,action))
    //         {
    //             std::shared_ptr<State> next_state = std::static_pointer_cast<State>(next_state_tmp);

    //             for(const auto &obst_tmp : pomdp->getReachableObservations(state,action,next_state))
    //             {
    //                 std::shared_ptr<Observation> obs = std::static_pointer_cast<Observation>(obst_tmp);

    //                 std::cout<< state->str()<<" , "<<action->str()<<" , "<<next_state->str()<<" , "<<obs->str()<<" , Reward = " << pomdp->getReward(state, action) << std::endl;
    //                 std::cout<<"Observation Probability : "<<pomdp->getObservationProbability(state,action,next_state,obs)<<std::endl;
    //                 std::cout<<"Dynamics : "<<pomdp->getDynamics(state,action,next_state,obs)<<std::endl;
    //             }
    //         }
    //     }
    // }

    // Creation of HSVI problem and Resolution 
    std::shared_ptr<SolvableByHSVI> hsvi_mdp = std::make_shared<SolvableByMDP>(mdp);

    // horizon = horizon * mdp->getNumAgents();
    auto tabular_backup = std::make_shared<TabularBackup>(hsvi_mdp);

    auto init_lb = std::make_shared<BlindInitializer>(hsvi_mdp);
    auto init_ub = std::make_shared<MaxInitializer>(hsvi_mdp);


    auto lb = std::make_shared<PointSetValueFunction>(tabular_backup,horizon,init_lb);
    auto ub = std::make_shared<PointSetValueFunction>(tabular_backup,horizon,init_ub);

    // auto lb = std::make_shared<MappedValueFunction>(hsvi_mdp, horizon, -1000);
    // auto ub = std::make_shared<MappedValueFunction>(hsvi_mdp, horizon, 1000);

    auto algo = std::make_shared<HSVI>(hsvi_mdp, lb, ub, horizon, 0.01);
    algo->do_initialize();
    std::cout << *algo->getLowerBound() << std::endl;
    std::cout << *algo->getUpperBound() << std::endl;
    algo->do_solve();
    // std::cout << *algo->getLowerBound() << std::endl;
    // std::cout << *algo->getUpperBound() << std::endl;

} // END main

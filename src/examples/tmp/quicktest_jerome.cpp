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

#include <sdm/world/mdp.hpp>
#include <sdm/world/mmdp.hpp>
#include <sdm/world/solvable_by_mdp.hpp>
#include <sdm/world/serialized_mmdp.hpp>
#include <sdm/world/mpomdp.hpp>
#include <sdm/world/pomdp.hpp>
#include <sdm/world/belief_mdp.hpp>

#include <sdm/parser/parser.hpp>

#include <sdm/utils/value_function/tabular_value_function.hpp>
// #include <sdm/utils/value_function/backup/tabular_backup.hpp>

#include <sdm/utils/value_function/initializer/initializer.hpp>
#include <sdm/utils/value_function/initializer/mdp_initializer.hpp>

#include <sdm/world/serialized_mpomdp.hpp>

using namespace sdm;

#include "sys/types.h"
#include "sys/sysinfo.h"

using namespace std;

long RanMemoryUsed(struct sysinfo memInfo)
{
    sysinfo (&memInfo);

    long long totalPhysMem = memInfo.totalram;
    //Multiply in next statement to avoid int overflow on right hand side...
    totalPhysMem *= memInfo.mem_unit;

    long long physMemUsed = memInfo.totalram - memInfo.freeram;
    //Multiply in next statement to avoid int overflow on right hand side...
    physMemUsed *= memInfo.mem_unit;

    std::cout<<"Total Ran Mem "<<totalPhysMem<<std::endl;
    std::cout<<"Ran Memory used  "<<physMemUsed<<std::endl;

    std::cout<<"Ratio : "<<100*physMemUsed/totalPhysMem<<std::endl;

    return physMemUsed;
}

int main(int argc, char **argv)
{
    struct sysinfo memInfo;
    auto start_memory = RanMemoryUsed(memInfo);

    auto mdp_tiger = sdm::parser::parse_file("../data/world/dpomdp/tiger.dpomdp");

    auto end_memory = RanMemoryUsed(memInfo);

    std::cout<<"Used Memory : "<<end_memory-start_memory<<std::endl;

    auto state_space = mdp_tiger->getStateSpace(); //std::make_shared<DiscreteSpace>(std::vector<std::shared_ptr<Item>>{state_0, state_1, state_2, state_3});
    auto action_space = mdp_tiger->getActionSpace(); //= std::make_shared<MultiDiscreteSpace>(std::vector<std::shared_ptr<Space>>{single_action_space, single_action_space});
    auto rew = mdp_tiger->getReward(); 
    auto dynamics = mdp_tiger->getStateDynamics();

    auto start_distrib = mdp_tiger->getStartDistribution();
    auto obs_space = mdp_tiger->getObservationSpace(0);
    auto obs_dynamics = mdp_tiger->getObservationDynamics();

    number horizon = 5;

    // Creation of the MMDP
    auto mdp = std::make_shared<MPOMDP>(state_space, action_space,obs_space, rew, dynamics,obs_dynamics,start_distrib,horizon,1.);

//     // Creation of the Serial MMDP with the MMDP
    auto serial_mmdp = std::make_shared<SerializedMPOMDP>(mdp);

//    mem_usage(vm, rss);
//    cout << "Virtual Memory: " << vm << "\nResident set size: " << rss << endl;

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

    auto init_lb = std::make_shared<MinInitializer>(hsvi_mdp);
    auto init_ub = std::make_shared<MaxInitializer>(hsvi_mdp);
    // auto init_ub = std::make_shared<MDPInitializer>(hsvi_mdp, "Hsvi",0.01,1000);

    auto lb = std::make_shared<TabularValueFunction>(horizon,init_lb,tabular_backup);
    auto ub = std::make_shared<TabularValueFunction>(horizon,init_ub,tabular_backup);

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

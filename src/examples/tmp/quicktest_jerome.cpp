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
#include <sdm/world/occupancy_mdp.hpp>

#include <sdm/parser/parser.hpp>

#include <sdm/utils/value_function/tabular_value_function.hpp>
#include <sdm/utils/value_function/point_set_value_function.hpp>
#include <sdm/utils/value_function/hyperplan_value_function.hpp>

#include <sdm/utils/value_function/backup/maxplan_backup.hpp>
#include <sdm/utils/value_function/backup/tabular_backup.hpp>

#include <sdm/utils/value_function/action_vf/action_tabulaire.hpp>
#include <sdm/utils/value_function/action_vf/action_maxplan_serial.hpp>
#include <sdm/utils/value_function/action_vf/action_maxplan.hpp>

#include <sdm/utils/value_function/evaluate_vf/evaluate_tabulair.hpp>
#include <sdm/utils/value_function/evaluate_vf/evaluate_sawtooth.hpp>
#include <sdm/utils/value_function/evaluate_vf/evaluate_maxplan.hpp>

#include <sdm/utils/value_function/initializer/initializer.hpp>
#include <sdm/utils/value_function/initializer/mdp_initializer.hpp>

#include <sdm/world/serialized_mpomdp.hpp>

#include <sdm/core/state/history_tree.hpp>
// #include <sdm/core/state/jhistory_tree.hpp>

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

    number horizon = 2;

    // Creation of the MMDP
    auto mpomdp = std::make_shared<MPOMDP>(state_space, action_space,obs_space, rew, dynamics,obs_dynamics,start_distrib,horizon,1.);
    // auto mmdp = std::make_shared<MMDP>(state_space, action_space, rew, dynamics,start_distrib,horizon,1.);

    // auto serial_mmdp = std::make_shared<SerializedMPOMDP>(mmdp);

    // Creation of HSVI problem and Resolution 
    // std::shared_ptr<SolvableByHSVI> hsvi = std::make_shared<SolvableByMDP>(mmdp);
    std::shared_ptr<SolvableByHSVI> hsvi = std::make_shared<OccupancyMDP>(mpomdp);

    // auto state = hsvi->getInitialState();

    // std::cout << "COMPRESSED \n"
    //             << state->str() << std::endl;

    // for (int i = 0; i < 2; i++)
    // {
    //     std::shared_ptr<Item> action;
    //     auto space = hsvi->getActionSpaceAt(state, i);
    //     for (const auto &decision_rule : *space)
    //     {
    //         action = decision_rule;
    //         std::cout<<"action"<<action->str()<<std::endl;
    //     }

    //     std::cout << "\n---- DECISION RULE ----- \n"
    //                 << action->str() << std::endl;
    //     state = hsvi->nextState(state, action->toAction(), i);
    //     std::cout << "\n---- COMPRESSED ----- \n"
    //                 << state->str() << std::endl;
    //     std::cout << "\n ----- ONE STEP UNCOMPRESSED -----\n"
    //                 << state->toOccupancyState()->getOneStepUncompressedOccupancy()->str() << std::endl;

    //     std::cout << "\n ----- FULLY UNCOMPRESSED -----\n"
    //                 << state->toOccupancyState()->getFullyUncompressedOccupancy()->str() << std::endl;
    // }

    // horizon = horizon * serial_mmdp->getNumAgents();
    auto tabular_backup = std::make_shared<TabularBackup>(hsvi);
    auto maxplan_backup = std::make_shared<MaxPlanBackup>(hsvi);

    auto evaluate_tabular = std::make_shared<EvaluateTabulaire>();
    auto evaluate_sawtooth = std::make_shared<EvaluateSawtooth>();
    auto evaluate_maxplan = std::make_shared<EvaluateMaxplan>();

    auto action_tabular = std::make_shared<ActionVFTabulaire>(hsvi);
    // auto action_sawtooth = std::make_shared<>(hsvi);
    auto action_maxplan = std::make_shared<ActionVFMaxplan>(hsvi);

    auto init_lb = std::make_shared<MinInitializer>(hsvi);
    auto init_ub = std::make_shared<MaxInitializer>(hsvi);

    auto ub = std::make_shared<TabularValueFunction>(horizon,init_ub,tabular_backup,action_tabular,evaluate_tabular);
    auto lb = std::make_shared<TabularValueFunction>(horizon,init_lb,tabular_backup,action_tabular,evaluate_tabular);

    auto algorithm = std::make_shared<HSVI>(hsvi, lb, ub, horizon, 0.01,2);
    algorithm->do_initialize();

    // std::cout << *algorithm->getLowerBound() << std::endl;
    // std::cout << *algorithm->getUpperBound() << std::endl;

    algorithm->do_solve(); 

    std::cout << *algorithm->getLowerBound() << std::endl;
    std::cout << *algorithm->getUpperBound() << std::endl;

    // for(const auto &state : *serial_mmdp->getStateSpace(1))
    // {
    //     auto serialized_state = state->toState()->toSerial();
    //     number agent_identifier = serialized_state->getCurrentAgentId();

    //     for(auto action_tmp : *serial_mmdp->getActionSpace(agent_identifier))
    //     {
    //         auto serial_action = action_tmp->toAction();

    //         // std::cout<<"Reward = " << serial_mmdp->getReward(state->toState(), serial_action,agent_identifier) << std::endl;

    //         for(const auto &next_state : serial_mmdp->getReachableStates(state->toState(),serial_action, agent_identifier+1))
    //         {
    //             // std::cout<<"Reachable State "<<next_state->str()<<", Transition Probability : "<<serial_mmdp->getTransitionProbability(state->toState(),serial_action,next_state->toState(),agent_identifier)<<std::endl;
    //             std::cout<<serialized_state->str()<<" , "<<serial_action->str()<<", "<<next_state->str()<<std::endl;

    //             for(const auto &next_obs : serial_mmdp->getReachableObservations(state->toState(),serial_action,next_state->toState(),agent_identifier+1))
    //             {
    //                 std::cout<<"Reachable Obs "<<next_obs->str()<<std::endl;
    //                 std::cout<<"Obs Probability : "<<serial_mmdp->getObservationProbability(state->toState(),serial_action,next_state->toState(),next_obs->toObservation(),agent_identifier)<<std::endl;
    //                 std::cout<<"Dynamics Probability : "<<serial_mmdp->getDynamics(state->toState(),serial_action,next_state->toState(),next_obs->toObservation(),agent_identifier)<<std::endl;

    //             }
    //         }
    //     }
    // }

    // std::cout << *algo->getLowerBound() << std::endl;
    // std::cout << *algo->getUpperBound() << std::endl;

    

    // std::cout<<algorithm->getUpperBound()->str()<<std::endl;
    // std::cout<<algorithm->getLowerBound()->str()<<std::endl;


    //  std::cout << "----- Usage : class Joint ( sdm/core/state/history_tree.hpp ) ---------" << std::endl
    //           << std::endl;

    // using TObservation = std::shared_ptr<Observation>;

    // number max_depth = 3;

    // std::shared_ptr<HistoryTree<TObservation>> history = std::make_shared<HistoryTree<TObservation>>(max_depth);

    // // Get basic elements of joint histories
    // std::cout << "\n--- 1) Basic access" << std::endl;

    // std::cout << "#> Horizon = " << history->getHorizon() << std::endl; // equivalent to history->getDepth()
    // std::cout << "#> MaxDepth = " << history->getMaxDepth() << std::endl;
    // std::cout << "#> Initial Joint history : " << history->str_not_const() << std::endl;

    // std::cout<<"getPtr : "<<history->getptr()<<std::endl;

    // // How to expand a joint history
    // std::cout << "\n--- 2) Instanciate and expand a history" << std::endl;

    // // List of joint observation for the example
    // auto obs_1 = std::make_shared<DiscreteObservation>(1);
    // auto obs_2 = std::make_shared<DiscreteObservation>(2);
    // std::shared_ptr<Action> action;

    // std::vector<std::shared_ptr<DiscreteObservation>> vec_obs = {obs_1,obs_2};

    // for (const auto &obs : vec_obs)
    // {
    //     std::cout << "\n#> Expand with observation " << obs->str() << std::endl;
    //     history = std::dynamic_pointer_cast<HistoryTree<TObservation>>(history->expand(obs, action));
    //     std::cout << "#> Expanded joint history --> " << history->str_not_const() << std::endl;
    // }

    // std::cout<<"\n get Last Observation "<<history->getData()<<std::endl;

    // std::cout<<"\n Get Parent of Joint History "<<history->getOrigin()->str_not_const()<<std::endl;

    // // How to access individual histories and expand them
    // std::cout << "\n--- 3) Access individual histories" << std::endl;

    // std::cout << "\n#> List of pointer on individual histories = " << history->getIndividualHistories() << std::endl;

    // for (number agent_id = 0; agent_id < history->getNumAgents(); ++agent_id)
    // {
    //     // std::cout << "#> IndividualHistory(" << agent_id << ") = " << *history->getIndividualHistory(agent_id) << std::endl; // equivalent to history->get(agent_id)
    // }

    return 0;


} // END main

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
#include <sdm/utils/value_function/point_set_value_function.hpp>
#include <sdm/utils/value_function/hyperplan_value_function.hpp>

// #include <sdm/utils/value_function/backup/tabular_backup.hpp>

#include <sdm/utils/value_function/initializer/initializer.hpp>
#include <sdm/utils/value_function/initializer/mdp_initializer.hpp>

#include <sdm/world/serialized_mpomdp.hpp>

#include <sdm/core/state/history_tree.hpp>

using namespace sdm;


int main(int argc, char **argv)
{
    // auto mdp_tiger = sdm::parser::parse_file("../data/world/dpomdp/tiger.dpomdp");

    // auto state_space = std::static_pointer_cast<DiscreteSpace>(mdp_tiger->getStateSpace()); //std::make_shared<DiscreteSpace>(std::vector<std::shared_ptr<Item>>{state_0, state_1, state_2, state_3});
    // auto action_space = mdp_tiger->getActionSpace(); //= std::make_shared<MultiDiscreteSpace>(std::vector<std::shared_ptr<Space>>{single_action_space, single_action_space});
    // auto rew = mdp_tiger->getReward(); 
    // auto dynamics = mdp_tiger->getStateDynamics();

    // auto start_distrib = mdp_tiger->getStartDistribution();
    // auto obs_space = mdp_tiger->getObservationSpace(0);
    // auto obs_dynamics = mdp_tiger->getObservationDynamics();

    // number horizon = 5;

    // // Creation of the MMDP
    // auto mpodp = std::make_shared<POMDP>(state_space, action_space,obs_space, rew, dynamics,obs_dynamics,start_distrib,horizon,1.);

    // // Creation of HSVI problem and Resolution 
    // std::shared_ptr<SolvableByHSVI> hsvi_mdp = std::make_shared<BeliefMDP>(mpodp);

    // std::cout<<"Test ? "<<std::endl;

    // // horizon = horizon * mdp->getNumAgents();
    // auto tabular_backup = std::make_shared<TabularBackup>(hsvi_mdp);

    // auto init_lb = std::make_shared<MinInitializer>(hsvi_mdp);
    // auto init_ub = std::make_shared<MaxInitializer>(hsvi_mdp);

    // auto ub = std::make_shared<TabularValueFunction>(horizon,init_ub,tabular_backup);
    // auto lb = std::make_shared<TabularValueFunction>(horizon,init_lb,tabular_backup);

    // auto algorithm = std::make_shared<HSVI>(hsvi_mdp, lb, ub, horizon, 0.01);
    // algorithm->do_initialize();
    // algorithm->do_solve();



    // Creation of the MMDP
    // auto mpodp = std::make_shared<POMDP>(state_space, action_space,obs_space, rew, dynamics,obs_dynamics,start_distrib,horizon,1.);

    // // Creation of HSVI problem and Resolution 
    // std::shared_ptr<SolvableByHSVI> hsvi_mdp = std::make_shared<BeliefMDP>(mpodp);

    // // horizon = horizon * mdp->getNumAgents();
    // auto tabular_backup = std::make_shared<TabularBackup>(hsvi_mdp);

    // auto ub = std::make_shared<HyperplanValueFunction>(horizon,1000,tabular_backup);

    // auto state = std::static_pointer_cast<BeliefMDP>(hsvi_mdp)->getInitialState()->toBelief();

    // std::cout<<" ******* Hyperplan Value Function ******"<<std::endl;

    // std::cout<<"Get Support "<<ub->getSupport(0)<<std::endl;
    // std::cout<<"Get isFinite "<<ub->isFiniteHorizon()<<std::endl;

    // std::cout<<"Max At before update "<<tabular_backup->getMaxAt(ub,state,0)<<std::endl;
    // std::cout<<"Backup "<<tabular_backup->backup(ub,state,0)<<std::endl;
    // ub->updateValueAt(state);
    // std::cout<<"Max At after update "<<tabular_backup->getMaxAt(ub,state,0)<<std::endl;
    // std::cout<<"Get Support "<<ub->getSupport(0)<<std::endl;

    // std::cout<<"Best Action "<<tabular_backup->getBestAction(ub,state,0)->str()<<std::endl;


     std::cout << "----- Usage : class Joint ( sdm/core/state/history_tree.hpp ) ---------" << std::endl
              << std::endl;

    using TObservation = std::shared_ptr<Observation>;

    number max_depth = 3;

    std::shared_ptr<HistoryTree<TObservation>> history = std::make_shared<HistoryTree<TObservation>>(max_depth);

    // Get basic elements of joint histories
    std::cout << "\n--- 1) Basic access" << std::endl;

    std::cout << "#> Horizon = " << history->getHorizon() << std::endl; // equivalent to history->getDepth()
    std::cout << "#> MaxDepth = " << history->getMaxDepth() << std::endl;
    std::cout << "#> Initial Joint history : " << history->str() << std::endl;

    std::cout<<"getPtr : "<<history->getptr()<<std::endl;

    // How to expand a joint history
    std::cout << "\n--- 2) Instanciate and expand a history" << std::endl;

    // List of joint observation for the example
    auto obs_1 = std::make_shared<DiscreteObservation>(1);
    auto obs_2 = std::make_shared<DiscreteObservation>(2);
    std::shared_ptr<Action> action;

    std::vector<std::shared_ptr<DiscreteObservation>> vec_obs = {obs_1,obs_2};

    for (const auto &obs : vec_obs)
    {
        std::cout << "\n#> Expand with observation " << obs->str() << std::endl;
        history = std::static_pointer_cast<HistoryTree<TObservation>>(history->expand(obs, action));
        std::cout << "#> Expanded joint history --> " << *history << std::endl;
    }

    std::cout<<"\n get Last Observation "<<history->getData()<<std::endl;

    std::cout<<"\n Get Parent of Joint History "<<*history->getOrigin()<<std::endl;

    // How to access individual histories and expand them
    std::cout << "\n--- 3) Access individual histories" << std::endl;

    // std::cout << "\n#> List of pointer on individual histories = " << history->getIndividualHistories() << std::endl;

    // for (number agent_id = 0; agent_id < history->getNumAgents(); ++agent_id)
    {
        // std::cout << "#> IndividualHistory(" << agent_id << ") = " << *history->getIndividualHistory(agent_id) << std::endl; // equivalent to history->get(agent_id)
    }

    return 0;


} // END main
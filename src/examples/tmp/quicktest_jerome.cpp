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


#include <sdm/utils/value_function/tabular_value_function.hpp>

using namespace sdm;

int main(int argc, char **argv)
{
    // Creation of State 
    auto state_0 = std::make_shared<DiscreteState>(0);
    auto state_1 = std::make_shared<DiscreteState>(1);
    auto state_2 = std::make_shared<DiscreteState>(2);
    auto state_3 = std::make_shared<DiscreteState>(3);

    // Creation of Action
    auto action_0 = std::make_shared<DiscreteAction>(0);
    auto action_1 = std::make_shared<DiscreteAction>(1);
    auto action_2 = std::make_shared<DiscreteAction>(2);
    auto action_3 = std::make_shared<DiscreteAction>(3);

    auto observation_0 = std::make_shared<DiscreteObservation>(0);
    auto observation_1 = std::make_shared<DiscreteObservation>(1);
    
    // Creation of a Serial State
    auto joint_serial_action = Joint<std::shared_ptr<Action>>(std::vector<std::shared_ptr<Action>>({action_2}));
    auto serial_state = std::make_shared<SerializedState>(state_0,joint_serial_action);

    // Some Function of Serial State
    std::cout<<serial_state->str()<<std::endl;
    std::cout<<"n_agent "<<serial_state->getCurrentAgentId()<<std::endl;

    for(const auto &action : serial_state->getAction())
    {
        std::cout<<"action"<<*action<<std::endl;
    }

    //Creation of space for the MMDP
    auto state_space = std::make_shared<DiscreteSpace>(std::vector<std::shared_ptr<Item>>{state_0, state_1, state_2, state_3});

    std::shared_ptr<Space> single_action_space = std::make_shared<DiscreteSpace>(std::vector<std::shared_ptr<Item>>{action_0, action_1, action_2, action_3});
    std::shared_ptr<Space> action_space = std::make_shared<MultiDiscreteSpace>(std::vector<std::shared_ptr<Space>>{single_action_space, single_action_space});
    // std::shared_ptr<Space> action_space = std::make_shared<DiscreteSpace>(std::vector<std::shared_ptr<Item>>{action_0, action_1, action_2, action_3});


    std::shared_ptr<Space> single_observation_space = std::make_shared<DiscreteSpace>(std::vector<std::shared_ptr<Item>>{observation_0, observation_1});
    auto observation_space = std::make_shared<MultiDiscreteSpace>(std::vector<std::shared_ptr<Space>>{single_observation_space, single_observation_space});
    // auto observation_space = std::make_shared<DiscreteSpace>(std::vector<std::shared_ptr<Item>>{observation_0, observation_1});

    auto start_distrib = std::make_shared<DiscreteDistribution<std::shared_ptr<State>>>();
    for (const auto &state : *state_space)
    {
        start_distrib->setProbability(std::static_pointer_cast<State>(state), 1. / state_space->getNumItems());
    }

    auto rew = std::make_shared<TabularReward>();

    double r = 0;
    for (const auto &state : *state_space) // = state_space->begin(); state != state_space->end(); state = state_space->next())
    {
        r += 1;
        for (const auto &action : *action_space) //= action_space->begin(); action != action_space->end(); action = action_space->next())
        {
            r += 1;
            rew->setReward(std::static_pointer_cast<State>(state), std::static_pointer_cast<Action>(action), r);
        }
    }

    std::cout << *rew << std::endl;

    auto dynamics = std::make_shared<TabularStateDynamics>();

    double proba = 1. / state_space->getNumItems();

    for (const auto &state : *state_space) // = state_space->begin(); state != state_space->end(); state = state_space->next())
    {
        for (const auto &action : *action_space) //= action_space->begin(); action != action_space->end(); action = action_space->next())
        {
            for (const auto &next_state : *state_space) // = state_space->begin(); state != state_space->end(); state = state_space->next())
            {
                dynamics->setTransitionProbability(std::static_pointer_cast<State>(state), std::static_pointer_cast<Action>(action), std::static_pointer_cast<State>(next_state), proba);
            }
        }
    }

    auto obs_dynamics = std::make_shared<TabularObservationDynamics>();

    proba = 1. / observation_space->getNumItems();

    for (const auto &state : *state_space)
    {
        for (const auto &action : *action_space) 
        {
            for (const auto &next_state : *state_space) 
            {
                for(const auto &obs : *observation_space)
                {
                    obs_dynamics->setObservationProbability(std::static_pointer_cast<State>(state), std::static_pointer_cast<Action>(action), std::static_pointer_cast<State>(next_state), std::static_pointer_cast<Observation>(obs),proba);
                    obs_dynamics->setDynamics(std::static_pointer_cast<State>(state), std::static_pointer_cast<Action>(action), std::static_pointer_cast<State>(next_state), std::static_pointer_cast<Observation>(obs),proba*dynamics->getTransitionProbability(std::static_pointer_cast<State>(state), std::static_pointer_cast<Action>(action), std::static_pointer_cast<State>(next_state)));
                }
            }
        }
    }

    number horizon = 3;

    //Creation of the MMDP
    auto pomdp = std::make_shared<MPOMDP>(horizon, 0.9, state_space, action_space,observation_space, rew, dynamics,obs_dynamics ,start_distrib);

    // Creation of the Serial MMDP with the MMDP
    // auto serial_mmdp = std::make_shared<SerializedMMDP>(mdp);

    // Some function of the Serial MMDP 
    std::cout << "#> NumAgents = " << pomdp->getNumAgents() << std::endl;
    std::cout << "#> Discount = " << pomdp->getDiscount() << std::endl;
    // std::cout << "#> isLastAgent(0)"<<pomdp->isLastAgent(0) << std::endl;
    // std::cout << "#> getAgentId( t = 3)"<<pomdp->getAgentId(3) << std::endl;
    std::cout << "#> Initial Distribution "<<pomdp->getStartDistribution() << std::endl;


    for(const auto &state_tmp : *pomdp->getStateSpace(0))
    {
        std::shared_ptr<State> state = std::static_pointer_cast<State>(state_tmp);

        for(auto action_tmp : *pomdp->getActionSpace(0))
        {
            std::shared_ptr<Action> action = std::static_pointer_cast<Action>(action_tmp);

            for(const auto &next_state_tmp : pomdp->getReachableStates(state,action))
            {
                std::shared_ptr<State> next_state = std::static_pointer_cast<State>(next_state_tmp);

                for(const auto &obst_tmp : pomdp->getReachableObservations(state,action,next_state))
                {
                    std::shared_ptr<Observation> obs = std::static_pointer_cast<Observation>(obst_tmp);

                    std::cout<< state->str()<<" , "<<action->str()<<" , "<<next_state->str()<<" , "<<obs->str()<<" , Reward = " << pomdp->getReward(state, action) << std::endl;
                    std::cout<<"Observation Probability : "<<pomdp->getObservationProbability(state,action,next_state,obs)<<std::endl;
                    std::cout<<"Dynamics : "<<pomdp->getDynamics(state,action,next_state,obs)<<std::endl;
                }
            }
        }
    }

    // Creation of HSVI problem and Resolution 
    // std::shared_ptr<SolvableByHSVI> hsvi_pomdp = std::make_shared<BeliefMDP>(pomdp);

    // horizon = horizon * pomdp->getNumAgents();
    // auto lb = std::make_shared<MappedValueFunction>(hsvi_pomdp, horizon, -1000);
    // auto ub = std::make_shared<MappedValueFunction>(hsvi_pomdp, horizon, 1000);

    // auto algo = std::make_shared<HSVI>(hsvi_pomdp, lb, ub, horizon, 0.01);
    // algo->do_initialize();
    // std::cout << *algo->getLowerBound() << std::endl;
    // std::cout << *algo->getUpperBound() << std::endl;
    // algo->do_solve();
    // std::cout << *algo->getLowerBound() << std::endl;
    // std::cout << *algo->getUpperBound() << std::endl;

} // END main

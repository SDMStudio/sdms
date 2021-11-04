#include <iostream>

#include <memory>
#include <sdm/exception.hpp>

#include <sdm/core/action/action.hpp>
#include <sdm/core/action/base_action.hpp>
#include <sdm/core/state/state.hpp>
#include <sdm/core/state/base_state.hpp>
#include <sdm/core/state/serial_state.hpp>

#include <sdm/core/space/discrete_space.hpp>
#include <sdm/core/space/multi_discrete_space.hpp>
#include <sdm/core/distribution.hpp>
#include <sdm/core/reward/tabular_reward.hpp>
#include <sdm/core/dynamics/tabular_state_dynamics.hpp>

#include <sdm/world/mdp.hpp>
#include <sdm/world/mmdp.hpp>
#include <sdm/world/solvable_by_mdp.hpp>
#include <sdm/world/serial_mmdp.hpp>

#include <sdm/utils/value_function/vfunction/tabular_value_function.hpp>

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
    
    // Creation of a Serial State
    auto joint_serial_action = Joint<std::shared_ptr<Action>>(std::vector<std::shared_ptr<Action>>({action_2}));
    auto serial_state = std::make_shared<SerialState>(state_0,joint_serial_action);

    // Some Function of Serial State
    std::cout<<serial_state->str()<<std::endl;
    std::cout<<"n_agent "<<serial_state->getCurrentAgentId()<<std::endl;

    for(const auto &action : serial_state->getAction())
    {
        std::cout<<"action"<<*action<<std::endl;
    }

    //Creation of space for the MMDP
    auto state_space = std::make_shared<DiscreteSpace>(std::vector<std::shared_ptr<Item>>{state_0, state_1, state_2, state_3});
    std::cout << *state_space << std::endl;

    std::shared_ptr<Space> single_action_space = std::make_shared<DiscreteSpace>(std::vector<std::shared_ptr<Item>>{action_0, action_1, action_2, action_3});
    std::shared_ptr<Space> action_space = std::make_shared<MultiDiscreteSpace>(std::vector<std::shared_ptr<Space>>{single_action_space, single_action_space});

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

    number horizon = 3;

    //Creation of the MMDP
    auto mdp = std::make_shared<MMDP>(state_space, action_space, rew, dynamics,start_distrib,horizon,1.);

    //Creation of the Serial MMDP with the MMDP
    auto serial_mmdp = std::make_shared<SerialMMDP>(mdp);

    // Some function of the Serial MMDP 
    std::cout << "#> NumAgents = " << serial_mmdp->getNumAgents() << std::endl;
    std::cout << "#> Discount = " << serial_mmdp->getDiscount() << std::endl;
    std::cout << "#> isLastAgent(0)"<<serial_mmdp->isLastAgent(0) << std::endl;
    std::cout << "#> getAgentId( t = 3)"<<serial_mmdp->getAgentId(3) << std::endl;
    // std::cout << "#> Initial Distribution "<<serial_mmdp->getStartDistribution() << std::endl;

    for(const auto &state : *serial_mmdp->getStateSpace(1))
    {
        auto serial_state = state->toState()->toSerial();
        number agent_identifier = serial_state->getCurrentAgentId();

        for(auto action_tmp : *serial_mmdp->getActionSpace(agent_identifier))
        {
            std::cout<<"action "<<action_tmp->str()<<std::endl;
            auto serial_action = action_tmp->toAction();

            std::cout<<serial_state->str()<<" , "<<serial_action->str()<<std::endl;
            std::cout<<"Reward = " << serial_mmdp->getReward(state->toState(), serial_action,agent_identifier) << std::endl;

            for(const auto &next_state : serial_mmdp->getReachableStates(state->toState(),serial_action))
            {
                std::cout<<"Reachable State "<<next_state->str()<<", Transition Probability : "<<serial_mmdp->getTransitionProbability(state->toState(),serial_action,next_state->toState(),agent_identifier)<<std::endl;
            }
        }
    }

} // END main

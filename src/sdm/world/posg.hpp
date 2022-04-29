#pragma once

#include <sdm/types.hpp>
#include <sdm/core/space/space.hpp>
#include <sdm/world/base/posg_interface.hpp>
#include <sdm/world/sg.hpp>
#include <sdm/world/mpomdp.hpp>

namespace sdm
{
    class POSG : virtual public POSGInterface, virtual public MPOMDP, virtual public SG
    {
    public:
        POSG();
        POSG(const std::shared_ptr<StateSpace> &state_space,
             const std::shared_ptr<ActionSpace> &action_space,
             const std::shared_ptr<ObservationSpace> &obs_space,
             const std::shared_ptr<RewardModel> &reward,
             const std::shared_ptr<StateDynamicsInterface> &state_dynamics,
             const std::shared_ptr<ObservationDynamicsInterface> &obs_dynamics,
             const std::shared_ptr<Distribution<std::shared_ptr<State>>> &start_distrib,
             number horizon = 0,
             double discount = 0.99,
             Criterion criterion = Criterion::REW_MAX);

        virtual ~POSG() {}

        /**
         * @brief Get the reward at timestep t when executing an action in a specific state.
         *
         * @param state the current state
         * @param action the action
         * @param t the timestep
         * @return double the reward for each agent
         */
        virtual double getReward(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t) const;

        /**
         * @brief Get the reward at timestep t when executing an action in a specific state.
         *
         * @param state the current state
         * @param action the action
         * @param t the timestep
         * @return double the reward for each agent
         */
        virtual double getReward(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number agent_id, number t) const;
    
        virtual std::string toStdFormat();
    };
}

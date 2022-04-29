
#pragma once

#include <sdm/types.hpp>
#include <sdm/core/space/space.hpp>
#include <sdm/world/base/sg_interface.hpp>
#include <sdm/world/mmdp.hpp>

namespace sdm
{
    /**
     * @brief The class for Stochastic Game.
     *
     */
    class SG : virtual public SGInterface, virtual public MMDP
    {
    public:
        SG();
        SG(const std::shared_ptr<StateSpace> &state_space,
           const std::shared_ptr<ActionSpace> &action_space,
           const std::shared_ptr<RewardModel> &reward,
           const std::shared_ptr<StateDynamicsInterface> &state_dynamics,
           const std::shared_ptr<Distribution<std::shared_ptr<State>>> &start_distrib,
           number horizon = 0,
           double discount = 0.99,
           Criterion criterion = Criterion::REW_MAX);

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
    };


} // namespace sdm

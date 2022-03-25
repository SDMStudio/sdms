/**
 * @file serial_mmdp.hpp
 * @author Jérôme ARJONILLA
 * @brief Defines the Serial MMDP.
 * @version 0.1
 * @date 17/08/2021
 *
 * @copyright Copyright (c) 2021
 *
 */

#pragma once

#include <sdm/types.hpp>
#include <sdm/utils/config.hpp>

#include <sdm/core/state/state.hpp>
#include <sdm/core/state/serial_state.hpp>

#include <sdm/core/action/action.hpp>
#include <sdm/core/distribution.hpp>
#include <sdm/world/base/mmdp_interface.hpp>
#include <sdm/world/serial_mmdp.hpp>

#include <sdm/core/space/multi_space.hpp>
#include <sdm/core/space/discrete_space.hpp>

#include <unordered_map>
#include <sdm/core/dynamics/tabular_state_dynamics.hpp>
#include <sdm/core/state/interface/serial_interface.hpp>

namespace sdm
{
    class ISerialMMDP : virtual public SerialMMDP
    {
    public:
        ISerialMMDP();
        ISerialMMDP(Config config);
        ISerialMMDP(const std::shared_ptr<MMDPInterface> &mmdp, Config config = {});

        virtual ~ISerialMMDP();

        /**
         * @brief Get the initial distribution over states.
         *
         * @return the initial distribution over states
         */
        std::shared_ptr<Distribution<std::shared_ptr<State>>> getStartDistribution() const;

        /**
         * @brief Get all states
         *
         * @return the set of states
         */
        std::shared_ptr<Space> getStateSpace(number t = 0) const;

        /**
         * @brief Get the reachable next states
         *
         * @param state the state
         * @param action the action
         * @return the set of reachable states
         */
        std::set<std::shared_ptr<State>> getReachableStates(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t = 0) const;

        /**
         * @brief Get the reward
         *
         * @param state
         * @param action
         * @param t
         * @return double
         */
        double getReward(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t = 0) const;

        /**
         * @brief Get the Transition Probability object
         *
         * @param state
         * @param action
         * @param next_state
         * @param t
         * @return double
         */
        double getTransitionProbability(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, number t = 0) const;

    protected:

        /**
         * @brief Refer to the Serial State Space
         *
         */
        Joint<std::shared_ptr<DiscreteSpace>> serial_state_space_;


        std::shared_ptr<Distribution<std::shared_ptr<State>>> distribution_serial;

        void createDistribution();
    };

} // namespace sdm
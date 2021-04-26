/**
 * @file decision_process.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief File that contains the implementation of the decision process class.
 * @version 1.0
 * @date 02/02/2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

#include <vector>
#include <tuple>

#include <sdm/types.hpp>
#include <sdm/exception.hpp>
#include <sdm/world/gym_interface.hpp>
#include <sdm/world/base/decision_process_base.hpp>

#include <sdm/core/space/discrete_space.hpp>
#include <sdm/core/space/multi_space.hpp>
#include <sdm/core/space/multi_discrete_space.hpp>
#include <sdm/core/state_dynamics.hpp>
#include <sdm/core/reward.hpp>

namespace sdm
{

    /**
     * @brief This class provides a way to instanciate multiple subclasses of decision processes.
     * 
     * @tparam TStateSpace the state space type
     * @tparam TActionSpace the action space type
     * @tparam TObsSpace the observation space type
     * @tparam TStateDynamics the state dynamics type
     * @tparam TReward the reward function type
     * @tparam TDistrib the type of the start distribution
     */
    template <typename TStateSpace, typename TActionSpace, typename TObsSpace, typename TStateDynamics, typename TReward, typename TDistrib, bool is_fully_obs = true>
    class DecisionProcess : public DecisionProcessBase<TStateSpace, TActionSpace, TDistrib>,
                            public GymInterface<typename TObsSpace::value_type, typename TActionSpace::value_type>
    {
    public:
        using state_type = typename DecisionProcessBase<TStateSpace, TActionSpace, TDistrib>::state_type;
        using observation_type = typename GymInterface<typename TObsSpace::value_type, typename TActionSpace::value_type>::observation_type;
        using action_type = typename DecisionProcessBase<TStateSpace, TActionSpace, TDistrib>::action_type;

        DecisionProcess();
        DecisionProcess(std::shared_ptr<TStateSpace>, std::shared_ptr<TActionSpace>);
        DecisionProcess(std::shared_ptr<TStateSpace>, std::shared_ptr<TActionSpace>, TDistrib);
        DecisionProcess(std::shared_ptr<TStateSpace>, std::shared_ptr<TActionSpace>, std::shared_ptr<TStateDynamics>, std::shared_ptr<TReward>, TDistrib, number = 0, double = 0.9, Criterion = Criterion::REW_MAX);
        DecisionProcess(std::shared_ptr<TStateSpace>, std::shared_ptr<TActionSpace>, std::shared_ptr<TObsSpace>, std::shared_ptr<TStateDynamics>, std::shared_ptr<TReward>, TDistrib, number = 0, double = 0.9, Criterion = Criterion::REW_MAX);
        virtual ~DecisionProcess();

        /**
         * \brief Get the state dynamics
         */
        std::shared_ptr<TStateDynamics> getStateDynamics() const;

        /**
         * \brief Set the state dynamics
         */
        void setStateDynamics(std::shared_ptr<TStateDynamics>);

        /**
         * \brief Get the reward function
         */
        std::shared_ptr<TReward> getReward() const;

        double getReward(state_type, action_type);

        /**
         * \brief Set the reward function
         */
        void setReward(std::shared_ptr<TReward>);

        /**
         * @brief Reset the process to initial settings.
         * 
         * @return the initial state (which is the internal state)
         */
        observation_type reset();

        std::tuple<observation_type, std::vector<double>, bool> step(action_type);

        /**
         * @brief Get the distribution over next states
         * 
         * @param cstate the current state
         * @param caction the current action
         * @return the distribution over next states
         */
        TDistrib getNextStateDistrib(state_type, action_type);

        /**
         * @brief Get the Next State Distrib object
         * @param state_type
         * @param action_type
         * @return const std::set<state_type>& 
         */
        const std::set<state_type>& getNextStateDistrib(state_type, action_type) const;

        /**
         * @brief Get the distribution over next states
         * @warning : Quand est-ce que cette fonction peut-elle servir?
         * 
         * @param cstate the current state
         * @param caction the current action
         * @return the distribution over next states
         */
        TDistrib getNextStateDistrib(action_type);

        template <bool TBool = is_fully_obs>
        std::enable_if_t<TBool, observation_type> updateState_getObs(action_type);
        
        virtual void setupDynamicsGenerator();

        // template <bool TBool = std::is_same<TDistrib, std::discrete_distribution<number>>::value>
        // std::enable_if_t<!TBool> setupDynamicsGenerator();

        std::shared_ptr<TActionSpace> getActionSpace() const;

        template <bool TBool = std::is_base_of<MultiSpace<DiscreteSpace<number>>, TActionSpace>::value>
        std::enable_if_t<TBool, number> getNumAgents();

        template <bool TBool = std::is_base_of<MultiSpace<DiscreteSpace<number>>, TActionSpace>::value>
        std::enable_if_t<!TBool, number> getNumAgents();

        std::shared_ptr<DiscreteSpace<action_type>> getActionSpaceAt(const observation_type &)
        {
            return this->getActionSpace();
        }

    protected:
        long ctimestep_ = 0;
        bool is_setup = false;

        /**
         * @brief State dynamics.
         */
        std::shared_ptr<TStateDynamics> state_dynamics_;

        /**
         * @brief Reward functions.
         */
        std::shared_ptr<TReward> reward_function_;

        /**
         * @brief Map (state, jaction) to probability of (next_state, next_observation) --> i.e. s_{t+1}, o_{t+1} ~ P(S_{t+1}, O_{t+1}  | S_t = s, A_t = a )
         */
        std::unordered_map<state_type, std::unordered_map<action_type, TDistrib>> dynamics_generator;

        /**
         * @brief Map (state, jaction) to Set of reachable next states
         */
        std::unordered_map<state_type, std::unordered_map<action_type, std::set<state_type>>> reachable_state_space;

        template <bool TBool = std::is_same<TActionSpace, MultiDiscreteSpace<number>>::value>
        std::enable_if_t<TBool, number>
        getAction(action_type);

        template <bool TBool = std::is_same<TActionSpace, MultiDiscreteSpace<number>>::value>
        std::enable_if_t<!TBool, action_type>
        getAction(action_type);

        template <bool TBool = is_fully_obs> //std::is_same<TStateSpace, TObsSpace>::value>
        std::enable_if_t<TBool, observation_type>
        resetProcess();

        template <bool TBool = is_fully_obs> //std::is_same<TStateSpace, TObsSpace>::value>
        std::enable_if_t<!TBool, observation_type>
        resetProcess();

        template <bool TBool = is_fully_obs> //std::is_same<TStateSpace, TObsSpace>::value>
        std::enable_if_t<TBool, std::tuple<observation_type, std::vector<double>, bool>>
        stepProcess(action_type);

        template <bool TBool = is_fully_obs> //std::is_same<TStateSpace, TObsSpace>::value>
        std::enable_if_t<!TBool, std::tuple<observation_type, std::vector<double>, bool>>
        stepProcess(action_type);
    };

    template <typename TStateSpace, typename TActionSpace, typename TStateDynamics, typename TReward, typename TDistrib>
    using FullyObservableDecisionProcess = DecisionProcess<TStateSpace, TActionSpace, TStateSpace, TStateDynamics, TReward, TDistrib, true>;

    using DiscreteSG = FullyObservableDecisionProcess<DiscreteSpace<number>, MultiDiscreteSpace<number>, StateDynamics, std::vector<Reward>, std::discrete_distribution<number>>;
} // namespace sdm
#include <sdm/world/decision_process.tpp>

/**
 * @file po_decision_process.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief File that define the PartiallyObservableDecisionProcess generic class.
 * @version 1.0
 * @date 26/02/2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

#include <sdm/types.hpp>
#include <sdm/world/decision_process.hpp>
#include <sdm/world/base/po_process_base.hpp>

#include <sdm/core/space/discrete_space.hpp>
#include <sdm/core/space/multi_discrete_space.hpp>
#include <sdm/core/state_dynamics.hpp>
#include <sdm/core/observation_dynamics.hpp>
#include <sdm/core/reward.hpp>

namespace sdm
{

    /**
     * @brief  This generic class allows to define a range of partially observable decision processes. 
     * 
     * @tparam TStateSpace the state space type
     * @tparam TActionSpace the action space type
     * @tparam TObsSpace the observation space type
     * @tparam TStateDynamics the state dynamics type
     * @tparam TObsDynamics the observation dynamics type
     * @tparam TReward the reward type
     * @tparam TDistrib the distribution type
     */
    template <typename TStateSpace, typename TActionSpace, typename TObsSpace, typename TStateDynamics, typename TObsDynamics, typename TReward, typename TDistrib>
    class PartiallyObservableDecisionProcess : public DecisionProcess<TStateSpace, TActionSpace, TObsSpace, TStateDynamics, TReward, TDistrib, false>,
                                               public PartiallyObservableProcessBase<TStateSpace, TObsSpace, TDistrib>
    {
    public:
        using state_type = typename DecisionProcessBase<TStateSpace, TActionSpace, TDistrib>::state_type;
        using observation_type = typename DecisionProcess<TStateSpace, TActionSpace, TObsSpace, TStateDynamics, TReward, TDistrib>::observation_type;
        using action_type = typename DecisionProcessBase<TStateSpace, TActionSpace, TDistrib>::action_type;
        using PartiallyObservableProcessBase<TStateSpace, TObsSpace, TDistrib>::getObsSpace;

        PartiallyObservableDecisionProcess();
        PartiallyObservableDecisionProcess(std::shared_ptr<TStateSpace>,
                                           std::shared_ptr<TActionSpace>,
                                           std::shared_ptr<TObsSpace>,
                                           std::shared_ptr<TStateDynamics>,
                                           std::shared_ptr<TObsDynamics>, 
                                           std::shared_ptr<TReward>,
                                           TDistrib,
                                           number = 0,
                                           double = 0.9,
                                           Criterion = Criterion::REW_MAX);

        PartiallyObservableDecisionProcess(PartiallyObservableDecisionProcess &);
        virtual ~PartiallyObservableDecisionProcess();

        /**
         * \brief Get the observation dynamics
         */
        std::shared_ptr<TObsDynamics> getObsDynamics() const;

        /**
         * \brief Set the observation dynamics
         */
        void setObsDynamics(std::shared_ptr<TObsDynamics>);

        observation_type updateState_getObs(action_type);
        
        std::tuple<observation_type, std::vector<double>, bool> step(action_type);

        /**
         * @brief Setup the dynamics generator for discrete problems. 
         * The dynamics generator allows to efficiently interact with the environment without recomputing transition probabilities at each timestep.
         */
        void setupDynamicsGenerator();

        /**
         * @brief Setup the dynamics generator for continuous problems. 
         * The dynamics generator allows to efficiently interact with the environment without recomputing transition probabilities at each timestep.
         */
        // template <bool TBool = std::is_same<TDistrib, std::discrete_distribution<number>>::value>
        // std::enable_if_t<!TBool> setupDynamicsGenerator();

    protected:
        /**
         * @brief State dynamics.
         */
        std::shared_ptr<TObsDynamics> obs_dynamics_;

        std::unordered_map<state_type, >

        /**
         * @brief map integer representing joint state/observation to this couple (state, observation)
         */
        std::unordered_map<number, std::pair<state_type, observation_type>> encoding;

        template <bool TBool = std::is_same<TObsSpace, MultiDiscreteSpace<number>>::value>
        std::enable_if_t<TBool, number>
        getObservation(observation_type);

        template <bool TBool = std::is_same<TObsSpace, MultiDiscreteSpace<number>>::value>
        std::enable_if_t<!TBool, observation_type>
        getObservation(observation_type);

    };

    template <typename TStateSpace, typename TActionSpace, typename TObsSpace, typename TStateDynamics, typename TObsDynamics, typename TReward, typename TDistrib>
    using PODecisionProcess = PartiallyObservableDecisionProcess<TStateSpace, TActionSpace, TObsSpace, TStateDynamics, TObsDynamics, TReward, TDistrib>;

    using DiscretePOSG = PODecisionProcess<DiscreteSpace<number>, MultiDiscreteSpace<number>, MultiDiscreteSpace<number>, StateDynamics, ObservationDynamics, std::vector<Reward>, std::discrete_distribution<number>>;
} // namespace sdm
#include <sdm/world/po_decision_process.tpp>

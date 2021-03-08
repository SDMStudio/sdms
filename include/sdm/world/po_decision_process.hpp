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
    class PartiallyObservableDecisionProcess : public DecisionProcess<TStateSpace, TActionSpace, TObsSpace, TStateDynamics, TReward, TDistrib>,
                                               public PartiallyObservableProcessBase<TStateSpace, TObsSpace, TDistrib>
    {
    public:
        using state_type = typename DecisionProcessBase<TStateSpace, TActionSpace, TDistrib>::state_type;
        using observation_type = typename PartiallyObservableProcessBase<TStateSpace, TObsSpace, TDistrib>::observation_type;
        using action_type = typename DecisionProcessBase<TStateSpace, TActionSpace, TDistrib>::action_type;
        using PartiallyObservableProcessBase<TStateSpace, TObsSpace, TDistrib>::getObsSpace;

        PartiallyObservableDecisionProcess();
        PartiallyObservableDecisionProcess(std::shared_ptr<TStateSpace> state_sp,
                                           std::shared_ptr<TActionSpace> action_sp,
                                           std::shared_ptr<TObsSpace> obs_sp,
                                           std::shared_ptr<TStateDynamics> state_dyn,
                                           std::shared_ptr<TObsDynamics> obs_dyn, std::shared_ptr<TReward>,
                                           TDistrib start_distrib,
                                           number planning_horizon = 0,
                                           double discount = 0.9,
                                           Criterion criterion = Criterion::REW_MAX);

        PartiallyObservableDecisionProcess(PartiallyObservableDecisionProcess &copy);

        /**
         * \brief Get the observation dynamics
         */
        std::shared_ptr<TObsDynamics> getObsDynamics() const;

        /**
         * \brief Set the observation dynamics
         */
        void setObsDynamics(std::shared_ptr<TObsDynamics> obs_dyn);

    protected:
        /**
         * @brief State dynamics.
         */
        std::shared_ptr<TObsDynamics> obs_dynamics_;

        /**
         * @brief map integer representing joint state/observation to this couple (state, observation)
         */
        std::unordered_map<number, std::pair<state_type, observation_type>> encoding;

    private:
        void setupDynamicsGenerator(std::true_type is_discrete, bool is_multi_agent)
        {
            int i = 0;
            for (auto &y : this->getStateSpace()->getAll())
            {
                for (auto &z : this->getObsSpace()->getAll())
                {
                    this->encoding.emplace(i, std::make_pair(y, z));
                    i++;
                }
            }

            for (auto &x : this->getStateSpace()->getAll())
            {
                this->dynamics_generator.emplace(x, std::unordered_map<action_type, TDistrib>());
                for (auto &a : this->getActionSpace()->getAll())
                {
                    std::vector<double> v;
                    for (auto &y : this->getStateSpace()->getAll())
                    {
                        for (auto &z : this->getObsSpace()->getAll())
                        {
                            if (is_multi_agent)
                            {
                                v.push_back(this->getObsDynamics()->getDynamics(x, this->getActionSpace()->joint2single(a), this->getObsSpace()->joint2single(z), y));
                            }
                            else
                            {
                                v.push_back(this->getObsDynamics()->getDynamics(x, a, z, y));
                            }
                        }
                    }
                    this->dynamics_generator[x].emplace(a, TDistrib(v.begin(), v.end()));
                }
            }
        }
    };

    template <typename TStateSpace, typename TActionSpace, typename TObsSpace, typename TStateDynamics, typename TObsDynamics, typename TReward, typename TDistrib>
    using PODecisionProcess = PartiallyObservableDecisionProcess<TStateSpace, TActionSpace, TObsSpace, TStateDynamics, TObsDynamics, TReward, TDistrib>;

    using DiscretePOSG = PODecisionProcess<DiscreteSpace<number>, MultiDiscreteSpace<number>, MultiDiscreteSpace<number>, StateDynamics, ObservationDynamics, std::vector<Reward>, std::discrete_distribution<number>>;
} // namespace sdm
#include <sdm/world/po_decision_process.tpp>

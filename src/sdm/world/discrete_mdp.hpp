/**
 * @file discrete_mdp.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief The file that contains the DiscreteMDP class.
 * @version 1.0
 * @date 02/02/2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

#include <sdm/types.hpp>
#include <sdm/world/solvable_by_hsvi.hpp>
#include <sdm/world/decision_process.hpp>

#include <sdm/core/space/discrete_space.hpp>
#include <sdm/core/state_dynamics.hpp>
#include <sdm/core/reward.hpp>

namespace sdm
{
    /**
     * @brief The class for Discrete Markov Decision Processes. 
     * 
     */
    class DiscreteMDP : public FullyObservableDecisionProcess<DiscreteSpace<number>, DiscreteSpace<number>, StateDynamics, Reward, std::discrete_distribution<number>>,
                        public SolvableByHSVI<number, number>
    {
    public:

        DiscreteMDP();
        DiscreteMDP(std::shared_ptr<DiscreteSpace<number>> state_sp, std::shared_ptr<DiscreteSpace<number>> action_sp);
        DiscreteMDP(std::shared_ptr<DiscreteSpace<number>> state_sp, std::shared_ptr<DiscreteSpace<number>> action_sp, std::discrete_distribution<number>);
        DiscreteMDP(std::shared_ptr<DiscreteSpace<number>> state_sp, std::shared_ptr<DiscreteSpace<number>> action_sp, std::shared_ptr<StateDynamics>, std::shared_ptr<Reward>, std::discrete_distribution<number> start_distrib, number planning_horizon = 0, double discount = 0.9, Criterion criterion = Criterion::REW_MAX);
        DiscreteMDP(std::string &filename);

        // SolvableByHSVI interface implementation
        number getInitialState();
        number nextState(const number &state, const number &action, int t = 0, HSVI<number, number> *hsvi = nullptr) const;
        double getDiscount();
        void setDiscount(double discount);
        std::shared_ptr<DiscreteSpace<number>> getActionSpaceAt(const number &state);
        std::shared_ptr<Reward> getReward() const;
        double getReward(const number &state, const number &action) const;
        double getExpectedNextValue(ValueFunction<number, number> *value_function, const number &state, const number &action, int t = 0) const;
    };
} // namespace sdm
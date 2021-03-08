/**
 * @file discrete_mmdp.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief 
 * @version 1.0
 * @date 05/03/2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

#include <sdm/types.hpp>
#include <sdm/world/decision_process.hpp>
#include <sdm/world/discrete_mdp.hpp>

#include <sdm/core/space/discrete_space.hpp>
#include <sdm/core/space/multi_discrete_space.hpp>
#include <sdm/core/state_dynamics.hpp>
#include <sdm/core/reward.hpp>

namespace sdm
{
    class DiscreteMMDP : public FullyObservableDecisionProcess<DiscreteSpace<number>, MultiDiscreteSpace<number>, StateDynamics, Reward, std::discrete_distribution<number>>
    {
    public:
        DiscreteMMDP();
        DiscreteMMDP(std::shared_ptr<DiscreteSpace<number>> state_sp, std::shared_ptr<MultiDiscreteSpace<number>> action_sp);
        DiscreteMMDP(std::shared_ptr<DiscreteSpace<number>> state_sp, std::shared_ptr<MultiDiscreteSpace<number>> action_sp, std::discrete_distribution<number>);
        DiscreteMMDP(std::shared_ptr<DiscreteSpace<number>> state_sp, std::shared_ptr<MultiDiscreteSpace<number>> action_sp, std::shared_ptr<StateDynamics>, std::shared_ptr<Reward>, std::discrete_distribution<number> start_distrib, number planning_horizon = 0, double discount = 0.9, Criterion criterion = Criterion::REW_MAX);

        std::shared_ptr<DiscreteMDP> toMDP();
    };
} // namespace sdm

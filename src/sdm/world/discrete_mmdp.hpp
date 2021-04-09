/**
 * @file discrete_mmdp.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief The file that contains the DiscreteMMDP class.
 * @version 1.0
 * @date 05/03/2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

#include <sdm/types.hpp>
#include <sdm/world/decision_process.hpp>
#include <sdm/world/belief_mdp.hpp>

#include <sdm/core/space/discrete_space.hpp>
#include <sdm/core/space/multi_discrete_space.hpp>
#include <sdm/core/state_dynamics.hpp>
#include <sdm/core/reward.hpp>

namespace sdm
{
    /**
     * @brief The class for Discrete Multi-agent Markov Decision Processes. 
     * 
     */
    class DiscreteMMDP : public FullyObservableDecisionProcess<DiscreteSpace<number>, MultiDiscreteSpace<number>, StateDynamics, Reward, std::discrete_distribution<number>>
    {
    public:
        DiscreteMMDP();
        DiscreteMMDP(std::shared_ptr<DiscreteSpace<number>> state_sp, std::shared_ptr<MultiDiscreteSpace<number>> action_sp);
        DiscreteMMDP(std::shared_ptr<DiscreteSpace<number>> state_sp, std::shared_ptr<MultiDiscreteSpace<number>> action_sp, std::discrete_distribution<number>);
        DiscreteMMDP(std::shared_ptr<DiscreteSpace<number>> state_sp, std::shared_ptr<MultiDiscreteSpace<number>> action_sp, std::shared_ptr<StateDynamics>, std::shared_ptr<Reward>, std::discrete_distribution<number> start_distrib, number planning_horizon = 0, double discount = 0.9, Criterion criterion = Criterion::REW_MAX);
        DiscreteMMDP(std::string &filename);

        /**
         * @brief Get the corresponding Markov Decision Process. It corresponds to the reformulation of the MMDP in a MDP where the Joint discrete action space becOme a discrete action Space . 
         * 
         * @return a belief MDP
        */
        std::shared_ptr<DiscreteMDP> toMDP();

        /**
         * @brief Get the corresponding Belief Markov Decision Process. Unfortunately, in this situation it isn't possible to transform a MMDP to a belief MDP  
         * 
         * @return a belief MDP
        */
        std::shared_ptr<BeliefMDP<BeliefState, number, number>> toBeliefMDP();
    };
} // namespace sdm

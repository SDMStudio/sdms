/**
 * @file discrete_pomdp.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief The file that contains the DiscretePOMDP class.
 * @version 1.0
 * @date 02/02/2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

#include <sdm/types.hpp>
#include <sdm/world/po_decision_process.hpp>
#include <sdm/world/discrete_mdp.hpp>
#include <sdm/world/belief_mdp.hpp>

#include <sdm/core/space/discrete_space.hpp>
#include <sdm/core/state_dynamics.hpp>
#include <sdm/core/reward.hpp>
#include <sdm/core/state/state.hpp>

namespace sdm
{
    /**
     * @brief The class for Discrete Partially Observable Markov Decision Processes. 
     * 
     */
    class DiscretePOMDP : public PartiallyObservableDecisionProcess<DiscreteSpace<number>,
                                                                    DiscreteSpace<number>,
                                                                    DiscreteSpace<number>,
                                                                    StateDynamics,
                                                                    ObservationDynamics,
                                                                    Reward,
                                                                    std::discrete_distribution<number>>,
                          public std::enable_shared_from_this<DiscretePOMDP>
    {
    public:
        DiscretePOMDP();
        DiscretePOMDP(std::string &filename);
        DiscretePOMDP(std::shared_ptr<DiscreteSpace<number>> state_sp, std::shared_ptr<DiscreteSpace<number>> action_sp, std::shared_ptr<DiscreteSpace<number>> obs_sp, std::shared_ptr<StateDynamics> state_dyn, std::shared_ptr<ObservationDynamics> obs_dyn, std::shared_ptr<Reward>, std::discrete_distribution<number> start_distrib, number planning_horizon = 0, double discount = 0.9, Criterion criterion = Criterion::REW_MAX);
        
        std::shared_ptr<DiscretePOMDP> getptr();

        /**
         * @brief Get the corresponding Markov Decision Process. It corresponds to the relaxation of the original POMP assuming that the agent can observation the state of the environment. 
         * 
         * @return a MDP 
         */
        std::shared_ptr<DiscreteMDP> toMDP();

        /**
         * @brief Get the corresponding Belief Markov Decision Process. It corresponds to the reformulation of the original POMP in a MDP where the state space is the space of beliefs. 
         * 
         * @return a belief MDP
         */
        std::shared_ptr<BeliefMDP<BeliefState, number, number>> toBeliefMDP();
    };
} // namespace sdm

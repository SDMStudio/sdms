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
#include <sdm/world/belief_mdp.hpp>

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
                        public SolvableByHSVI<number, number>,
                        public std::enable_shared_from_this<DiscreteMDP>
    {
    public:
        DiscreteMDP();
        DiscreteMDP(std::shared_ptr<DiscreteSpace<number>> , std::shared_ptr<DiscreteSpace<number>> );
        DiscreteMDP(std::shared_ptr<DiscreteSpace<number>> , std::shared_ptr<DiscreteSpace<number>> , std::discrete_distribution<number>);
        DiscreteMDP(std::shared_ptr<DiscreteSpace<number>> , std::shared_ptr<DiscreteSpace<number>> , std::shared_ptr<StateDynamics>, std::shared_ptr<Reward>, std::discrete_distribution<number> , number  = 0, double  = 0.9, Criterion  = Criterion::REW_MAX);
        DiscreteMDP(std::string &);

        std::shared_ptr<DiscreteMDP> getptr();
        std::shared_ptr<Reward> getReward() const;

        // SolvableByHSVI interface implementation
        number getInitialState();
        number nextState(const number &, const number &, number = 0, std::shared_ptr<HSVI<number, number>>  = nullptr) const;
        std::shared_ptr<DiscreteSpace<number>> getActionSpaceAt(const number &);
        double getReward(const number &, const number &) const;
        double getExpectedNextValue(ValueFunction<number, number> *, const number &, const number &, number = 0) const;
        DiscreteMDP *getUnderlyingProblem();
        bool isSerialized() const;

        // Problem conversion
        std::shared_ptr<DiscreteMDP> toMDP();

        /**
         * @brief Get the corresponding Belief Markov Decision Process. Unfortunately, in this situation it isn't possible to transform a MMDP to a belief MDP  
         * @warning The above claim is not true!!!!
         * 
         * @return a belief MDP
         */
        std::shared_ptr<BeliefMDP<BeliefState, number, number>> toBeliefMDP();
    };
} // namespace sdm
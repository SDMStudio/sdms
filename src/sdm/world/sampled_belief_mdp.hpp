/**
 * @file belief_mdp.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief File that contains the implementation of the belief mdp process class.
 * @version 1.0
 * @date 03/02/2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

#include <sdm/world/belief_mdp.hpp>

namespace sdm
{
    /**
     * @brief The SampledBaseBeliefMDP class is the interface contains the transformation of a the POMDP formalism in BaseBeliefMDP formalism with sampled beliefs.
     */
    template <class TBelief>
    class SampledBaseBeliefMDP : public BaseBeliefMDP<TBelief>

    {
    public:
        SampledBaseBeliefMDP();
        SampledBaseBeliefMDP(const std::shared_ptr<POMDPInterface> &pomdp, int batch_size);

        std::shared_ptr<Observation> reset();

        std::tuple<std::shared_ptr<Observation>, std::vector<double>, bool> step(std::shared_ptr<Action> action);

    protected:
        int batch_size_;
        int b_star_;
        std::vector<std::shared_ptr<State>> initial_x_vector_, current_x_vector_;
        std::vector<double> rewards_vector;

        virtual Pair<std::shared_ptr<State>, double> computeNextStateAndProba(const std::shared_ptr<State> &belief, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t = 0);
    };

    using SampledBeliefMDP = SampledBaseBeliefMDP<Belief>;

}
#include <sdm/world/sampled_belief_mdp.tpp>

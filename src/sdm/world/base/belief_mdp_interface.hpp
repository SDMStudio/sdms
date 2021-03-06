#pragma once

#include <sdm/types.hpp>
#include <sdm/exception.hpp>
#include <sdm/world/solvable_by_hsvi.hpp>
#include <sdm/world/base/pomdp_interface.hpp>
#include <sdm/utils/struct/graph.hpp>

namespace sdm
{
    /**
     * @brief Public interface for every belief mdp.
     */
    class BeliefMDPInterface : virtual public SolvableByHSVI
    {
    public:
        /** @brief Get the address of the underlying POMDP */
        virtual std::shared_ptr<POMDPInterface> getUnderlyingPOMDP() const = 0;

        /** @brief Get the address of the underlying BeliefMDP */
        virtual std::shared_ptr<BeliefMDPInterface> getUnderlyingBeliefMDP() = 0;

        /**
         * @brief Get the MDP graph. 
         * 
         * In the case where the variables `store_states` and `store_actions` are set to true, 
         * we iteratively construct and save the graph of state transitions. 
         * 
         * @return the graph representing a markov decision process 
         */
        virtual std::shared_ptr<Graph<std::shared_ptr<State>, Pair<std::shared_ptr<Action>, std::shared_ptr<Observation>>>> getMDPGraph() = 0;

        virtual double getObservationProbability(const std::shared_ptr<State> &belief, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_belief, const std::shared_ptr<Observation> &obs, number t = 0) const = 0;
    };
} // namespace sdm
#pragma once

#include <sdm/utils/struct/graph.hpp>
#include <sdm/world/belief_mdp.hpp>

namespace sdm
{
    /**
     * @brief A graph that keep all beliefs
     * 
     * @tparam TBeliefState 
     * @tparam TAction 
     * @tparam TObservation 
     */
    template <typename TBeliefState, typename TAction, typename TObservation>
    class BeliefStateGraph : public Graph<TBeliefState, Pair<TAction, TObservation>>
    {
    public:
        BeliefStateGraph();

        BeliefStateGraph(const std::shared_ptr<TBeliefState> &data);

        /**
         * @brief Construct a new belief 
         * 
         * @param predecessor 
         * @param belief 
         */
        BeliefStateGraph(const std::shared_ptr<BeliefStateGraph> &predecessor, const std::shared_ptr<TBeliefState> &belief);

        void setDynamics(std::vector<std::vector<Matrix>> dynamics);
        std::shared_ptr<TBeliefState> nextBelief(const TAction &action, const TObservation &observation);

        /**
         * @brief  Expands the graph
         * 
         * @param action the action
         * @param observation the observation
         * @param backup 
         * @return the next belief
         */
        template <typename output = BeliefStateGraph<TBeliefState, TAction, TObservation>>
        std::shared_ptr<output> expand(const TAction &action, const TObservation &observation, bool backup = true);

        std::string str();

        std::shared_ptr<BeliefStateGraph<TBeliefState, TAction, TObservation>> getptr();

        friend std::ostream &operator<<(std::ostream &os, BeliefStateGraph &belief_state_graph)
        {
            os << belief_state_graph.str();
            return os;
        }

    protected:
        /**
         * @brief The belief mdp problem. 
         */
        std::shared_ptr<std::vector<std::vector<Matrix>>> dynamics_;

        /**
         * @brief A pointer on the bag containing all nodes 
         */
        std::shared_ptr<std::unordered_map<std::shared_ptr<TBeliefState>, std::shared_ptr<BeliefStateGraph>>> belief_space;
    };

} // namespace sdm
#include <sdm/core/state/belief_state_graph.tpp>

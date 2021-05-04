#include <sdm/core/state/belief_state_graph.hpp>

namespace sdm
{

    template <typename TBeliefState, typename TAction, typename TObservation>
    BeliefStateGraph<TBeliefState, TAction, TObservation>::BeliefStateGraph()
        : Graph<TBeliefState, Pair<TAction, TObservation>>()
    {
    }

    template <typename TBeliefState, typename TAction, typename TObservation>
    BeliefStateGraph<TBeliefState, TAction, TObservation>::BeliefStateGraph(const std::shared_ptr<TBeliefState> &data)
        : Graph<TBeliefState, Pair<TAction, TObservation>>(data)
    {
        this->dynamics_ = std::make_shared<std::vector<std::vector<Matrix>>>();
    }

    template <typename TBeliefState, typename TAction, typename TObservation>
    BeliefStateGraph<TBeliefState, TAction, TObservation>::BeliefStateGraph(const std::shared_ptr<BeliefStateGraph<TBeliefState, TAction, TObservation>> &predecessor, const std::shared_ptr<TBeliefState> &belief)
        : Graph<TBeliefState, Pair<TAction, TObservation>>(predecessor, belief)
    {
    }

    template <typename TBeliefState, typename TAction, typename TObservation>
    void BeliefStateGraph<TBeliefState, TAction, TObservation>::setDynamics(std::vector<std::vector<Matrix>> dynamics)
    {
        *this->dynamics_ = dynamics;
    }

    template <typename TBeliefState, typename TAction, typename TObservation>
    std::shared_ptr<TBeliefState> BeliefStateGraph<TBeliefState, TAction, TObservation>::nextBelief(const TAction &action, const TObservation &observation)
    {
        auto next_belief = (this->dynamics_.at(action).at(observation) * (*this->getData()));
        return std::make_shared<TBeliefState>(next_belief);
    }

    template <typename TBeliefState, typename TAction, typename TObservation>
    template <typename output>
    std::shared_ptr<output> BeliefStateGraph<TBeliefState, TAction, TObservation>::expand(const TAction &action, const TObservation &observation, bool backup)
    {
        Pair<TAction, TObservation> pair_action_observation(action, observation);
        if (backup && (this->successors.find(pair_action_observation) != this->successors.end()))
        {
            // If already in the successor list, return the successor node
            return std::static_pointer_cast<output>(this->getSuccessor(pair_action_observation));
        }
        if (backup)
        {
            // Build next belief
            auto next_belief = this->nextBelief(action, observation);
            auto proba_next_belief = this->computeProbaNextBelief(action, observation);

            // Create a successor node
            auto node_ptr = std::make_shared<output>(std::static_pointer_cast<output>(this->getptr()), next_belief);

            // Add the belief in the space of beliefs
            this->belief_space.emplace(next_belief, node_ptr);

            // Add the sucessor in the list of successors
            this->successors.emplace(pair_action_observation, proba_next_belief, node_ptr);

            return std::static_pointer_cast<output>(this->getSuccessor(pair_action_observation));
        }
        // Return next belief without storing its value in the graph
        auto next_belief = this->nextBelief(action, observation);
        return std::make_shared<output>(std::static_pointer_cast<output>(this->getptr()), next_belief);
    }

    template <typename TBeliefState, typename TAction, typename TObservation>
    std::string BeliefStateGraph<TBeliefState, TAction, TObservation>::str()
    {
        std::ostringstream str_result;
        str_result << *this->getData();
        return str_result.str();
    }

    template <typename TBeliefState, typename TAction, typename TObservation>
    std::shared_ptr<BeliefStateGraph<TBeliefState, TAction, TObservation>> BeliefStateGraph<TBeliefState, TAction, TObservation>::getptr()
    {
        return std::static_pointer_cast<BeliefStateGraph<TBeliefState, TAction, TObservation>>(this->shared_from_this());
    }

} // namespace sdm

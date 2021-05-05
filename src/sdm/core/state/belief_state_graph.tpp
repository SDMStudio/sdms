#include <sdm/core/state/belief_state_graph.hpp>

namespace sdm
{

    template <typename TBeliefState, typename TAction, typename TObservation>
    BeliefStateGraph<TBeliefState, TAction, TObservation>::BeliefStateGraph()
        : Graph<TBeliefState, Pair<TAction, TObservation>>()
    {
    }

    template <typename TBeliefState, typename TAction, typename TObservation>
    BeliefStateGraph<TBeliefState, TAction, TObservation>::BeliefStateGraph(const TBeliefState &data, const std::vector<std::vector<Matrix>> &dynamics)
        : Graph<TBeliefState, Pair<TAction, TObservation>>(data)
    {
        this->dynamics_ = std::make_shared<std::vector<std::vector<Matrix>>>(dynamics);
        this->belief_space = std::make_shared<std::unordered_map<TBeliefState, std::shared_ptr<BeliefStateGraph>>>();
    }

    template <typename TBeliefState, typename TAction, typename TObservation>
    BeliefStateGraph<TBeliefState, TAction, TObservation>::BeliefStateGraph(const std::shared_ptr<BeliefStateGraph<TBeliefState, TAction, TObservation>> &predecessor, const TBeliefState &belief)
        : Graph<TBeliefState, Pair<TAction, TObservation>>(predecessor, belief), dynamics_(predecessor->dynamics_), belief_space(predecessor->belief_space)
    {
    }

    template <typename TBeliefState, typename TAction, typename TObservation>
    void BeliefStateGraph<TBeliefState, TAction, TObservation>::initialize()
    {
        this->belief_space->emplace(this->getData(), this->getptr());
    }

    template <typename TBeliefState, typename TAction, typename TObservation>
    std::shared_ptr<BeliefStateGraph<TBeliefState, TAction, TObservation>> BeliefStateGraph<TBeliefState, TAction, TObservation>::getNode(const TBeliefState &belief)
    {
        return this->belief_space->at(belief);
    }

    template <typename TBeliefState, typename TAction, typename TObservation>
    void BeliefStateGraph<TBeliefState, TAction, TObservation>::setDynamics(std::vector<std::vector<Matrix>> dynamics)
    {
        *this->dynamics_ = dynamics;
    }

    template <typename TBeliefState, typename TAction, typename TObservation>
    double BeliefStateGraph<TBeliefState, TAction, TObservation>::getProbability(const TAction &action, const TObservation &observation) const
    {
        return this->belief_proba.at(action).at(observation);
    }

    template <typename TBeliefState, typename TAction, typename TObservation>
    std::pair<TBeliefState, double> BeliefStateGraph<TBeliefState, TAction, TObservation>::computeNextBelief(const TAction &action, const TObservation &observation)
    {
        // Compute next coef belief (non normailized)
        auto weighted_next_belief = (this->dynamics_->at(action).at(observation) * this->getData());

        // Compute the coefficient of normalization (eta)
        double eta = weighted_next_belief.norm_1();

        return {(1. / eta) * weighted_next_belief, eta};
    }

    template <typename TBeliefState, typename TAction, typename TObservation>
    template <typename output>
    std::shared_ptr<output> BeliefStateGraph<TBeliefState, TAction, TObservation>::expand(const TAction &action, const TObservation &observation, bool backup)
    {
        auto pair_action_observation = std::make_pair(action, observation);
        // If already in the successor list
        if (backup && (this->successors.find(pair_action_observation) != this->successors.end()))
        {
            // Return the successor node
            return std::static_pointer_cast<output>(this->getSuccessor(pair_action_observation));
        }
        if (backup)
        {
            // Build next belief and proba
            auto [next_belief, proba_belief] = this->computeNextBelief(action, observation);

            // Store the probability of next belief
            this->belief_proba[action][observation] = proba_belief;

            // Get node on the next belief
            std::shared_ptr<output> node_ptr;
            if (this->belief_space->find(next_belief) == this->belief_space->end())
            {
                // Create a successor node
                node_ptr = std::make_shared<output>(std::static_pointer_cast<output>(this->getptr()), next_belief);
                
                // Add the belief in the space of beliefs
                this->belief_space->emplace(next_belief, node_ptr);
            }
            else
            {
                // Get the successor node
                node_ptr = this->belief_space->at(next_belief);
            }

            // Add the sucessor in the list of successors
            this->successors.emplace(pair_action_observation, node_ptr);

            return std::static_pointer_cast<output>(this->getSuccessor(pair_action_observation));
        }
        // Return next belief without storing its value in the graph
        auto [next_belief, proba_belief] = this->computeNextBelief(action, observation);
        return std::make_shared<output>(std::static_pointer_cast<output>(this->getptr()), next_belief);
    }

    template <typename TBeliefState, typename TAction, typename TObservation>
    std::string BeliefStateGraph<TBeliefState, TAction, TObservation>::str()
    {
        std::ostringstream str_result;
        str_result << this->getData();
        return str_result.str();
    }

    template <typename TBeliefState, typename TAction, typename TObservation>
    std::shared_ptr<BeliefStateGraph<TBeliefState, TAction, TObservation>> BeliefStateGraph<TBeliefState, TAction, TObservation>::getptr()
    {
        return std::static_pointer_cast<BeliefStateGraph<TBeliefState, TAction, TObservation>>(this->shared_from_this());
    }

} // namespace sdm

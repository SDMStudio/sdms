#include <sdm/core/state/belief_state_graph.hpp>
#include <sdm/exception.hpp>

namespace sdm
{
    BeliefStateGraph::BeliefStateGraph()
        : Graph<Belief, Pair<std::shared_ptr<Action>, std::shared_ptr<Observation>>>()
    {
        this->belief_space = std::make_shared<std::unordered_map<Belief, std::shared_ptr<BeliefStateGraph>>>();
    }

    BeliefStateGraph::BeliefStateGraph(const Belief &data)
        : Graph<Belief, Pair<std::shared_ptr<Action>, std::shared_ptr<Observation>>>(data)
    {
        this->belief_space = std::make_shared<std::unordered_map<Belief, std::shared_ptr<BeliefStateGraph>>>();
    }

    BeliefStateGraph::BeliefStateGraph(const std::vector<std::shared_ptr<State>> &list_states, const std::vector<double> &list_proba)
        : BeliefStateGraph(Belief(list_states, list_proba))
    {
    }

    BeliefStateGraph::BeliefStateGraph(const std::shared_ptr<BeliefStateGraph> &predecessor, const Belief &belief)
        : Graph<Belief, Pair<std::shared_ptr<Action>, std::shared_ptr<Observation>>>(predecessor, belief), belief_space(predecessor->belief_space)
    {
    }
    
    std::shared_ptr<BeliefStateGraph> BeliefStateGraph::getNode(const Belief &belief)
    {
        return this->belief_space->at(belief);
    }

    double BeliefStateGraph::getProbability(const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation) const
    {
        return this->belief_proba.at(action).at(observation);
    }

    std::shared_ptr<BeliefStateGraph> BeliefStateGraph::next(TransitionFunction transition_function, const std::shared_ptr<POMDPInterface> &pomdp, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t, bool backup)
    {
        auto pair_action_observation = std::make_pair(action, observation);
        // If already in the successor list
        if (backup && (this->successors.find(pair_action_observation) != this->successors.end()))
        {
            // Return the successor node
            return std::static_pointer_cast<BeliefStateGraph>(this->getSuccessor(pair_action_observation));
        }
        if (backup)
        {
            // Build next belief and proba
            auto [next_belief_p, proba_belief] = transition_function(pomdp, this->getptr(), action, observation, t);


            auto next_belief = *std::dynamic_pointer_cast<Belief>(next_belief_p);

            // Store the probability of next belief
            this->belief_proba[action][observation] = proba_belief;

            std::shared_ptr<BeliefStateGraph> node_ptr;

            // Get node on the next belief
            auto iterator = this->belief_space->find(next_belief);
            if (iterator == this->belief_space->end())
            {
                // Create a successor node
                node_ptr = std::make_shared<BeliefStateGraph>(std::static_pointer_cast<BeliefStateGraph>(this->getptr()), next_belief);

                // Add the belief in the space of beliefs
                this->belief_space->emplace(next_belief, node_ptr);
            }
            else
            {
                // Get the successor node
                node_ptr = iterator->second;
            }

            // Add the sucessor in the list of successors
            this->successors.emplace(pair_action_observation, node_ptr);

            return std::static_pointer_cast<BeliefStateGraph>(this->getSuccessor(pair_action_observation));
        }
        // Return next belief without storing its value in the graph
        auto [next_belief_p, proba_belief] = transition_function(pomdp, this->getptr(), action, observation, t);
        auto next_belief = *std::dynamic_pointer_cast<Belief>(next_belief_p);
        return std::make_shared<BeliefStateGraph>(std::static_pointer_cast<BeliefStateGraph>(this->getptr()), next_belief);
    }

    std::string BeliefStateGraph::str() const
    {
        std::ostringstream str_result;
        str_result << this->getData().str();
        return str_result.str();
    }

    std::shared_ptr<BeliefStateGraph> BeliefStateGraph::getptr()
    {
        return Graph<Belief, Pair<std::shared_ptr<Action>, std::shared_ptr<Observation>>>::downcasted_shared_from_this<BeliefStateGraph>();
    }

    template <class Archive>
    void BeliefStateGraph::serialize(Archive &archive, const unsigned int)
    {
        using boost::serialization::make_nvp;

        archive &boost::serialization::base_object<Graph<Belief, Pair<std::shared_ptr<Action>, std::shared_ptr<Observation>>>>(*this);
        archive &make_nvp("belief_space", belief_space);
        archive &make_nvp("belief_proba", belief_proba);
    }

    std::vector<std::shared_ptr<State>> BeliefStateGraph::getStates() const
    {
        return this->getData().getStates();
    }

    size_t BeliefStateGraph::size() const
    {
        return this->getData().size();
    }

    double BeliefStateGraph::getProbability(const std::shared_ptr<State> &state) const
    {
        return this->getData().getProbability(state);
    }

    void BeliefStateGraph::setProbability(const std::shared_ptr<State> &state, double proba)
    {
        this->data_.setProbability(state, proba);
    }

    void BeliefStateGraph::addProbability(const std::shared_ptr<State> &state, double proba)
    {
        this->data_.addProbability(state, proba);
    }

    bool BeliefStateGraph::operator==(const std::shared_ptr<BeliefInterface> &other) const
    {
        return this->getData().operator==(other);
    }
    double BeliefStateGraph::operator^(const std::shared_ptr<BeliefInterface> &other) const
    {
        return this->getData().operator^(other);
    }
    double BeliefStateGraph::norm_1() const
    {
        return this->getData().norm_1();
    }

    std::shared_ptr<VectorInterface<std::shared_ptr<State>,double>> BeliefStateGraph::getVectorInferface()
    {
        throw sdm::exception::NotImplementedException();
    }

    void BeliefStateGraph::setDefaultValue(double value)
    {
        this->data_.setDefaultValue(value);
    }
    double BeliefStateGraph::getDefaultValue() const
    {
        return this->getData().getDefaultValue();
    }

} // namespace sdm

#include <sdm/core/state/belief_state_graph.hpp>
#include <sdm/exception.hpp>

namespace sdm
{
    BeliefStateGraph::BeliefStateGraph()
        : Graph<std::shared_ptr<BeliefInterface>, Pair<std::shared_ptr<Action>, std::shared_ptr<Observation>>>()
    {
        this->belief_space = std::make_shared<std::unordered_map<std::shared_ptr<BeliefInterface>, std::shared_ptr<BeliefStateGraph>>>();
        // this->belief_pointer = std::make_shared<std::unordered_map<Belief, std::shared_ptr<BeliefInterface>>>();

    }

    BeliefStateGraph::BeliefStateGraph(const std::shared_ptr<BeliefInterface> &data)
        : Graph<std::shared_ptr<BeliefInterface>, Pair<std::shared_ptr<Action>, std::shared_ptr<Observation>>>(data)
    {
        this->belief_space = std::make_shared<std::unordered_map<std::shared_ptr<BeliefInterface>, std::shared_ptr<BeliefStateGraph>>>();
        // this->belief_pointer = std::make_shared<std::unordered_map<Belief, std::shared_ptr<BeliefInterface>>>();
    }

    // BeliefStateGraph::BeliefStateGraph(const std::vector<std::shared_ptr<State>> &list_states, const std::vector<double> &list_proba)
    //     : BeliefStateGraph(Belief(list_states, list_proba))
    // {
    // }

    BeliefStateGraph::BeliefStateGraph(const std::shared_ptr<BeliefStateGraph> &predecessor, const std::shared_ptr<BeliefInterface> &belief)
        : Graph<std::shared_ptr<BeliefInterface>, Pair<std::shared_ptr<Action>, std::shared_ptr<Observation>>>(predecessor, belief), belief_space(predecessor->belief_space)
    {
    }

    void BeliefStateGraph::initialize()
    {
        this->belief_space->emplace(this->getData(), this->getptr());
    }


    std::shared_ptr<BeliefStateGraph> BeliefStateGraph::getNode(const std::shared_ptr<BeliefInterface> &belief)
    {
        std::cout<<"Ici "<<std::endl;
        return this->belief_space->at(this->exist(belief));
    }

    double BeliefStateGraph::getProbability(const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation) const
    {
        return this->belief_probability.at(action).at(observation);
    }

    std::shared_ptr<BeliefStateGraph> BeliefStateGraph::next(const std::shared_ptr<BeliefInterface>&belief, double probability , const std::shared_ptr<POMDPInterface> &pomdp, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t, bool backup )
    {
        auto action_observation = std::make_pair(action, observation);

        if (backup)
        {   
            // If already in the successor list
            if ((this->successors.find(action_observation) != this->successors.end()))
            {
                // Return the successor node
                return std::static_pointer_cast<BeliefStateGraph>(this->getSuccessor(action_observation));
            }
            else
            {
                // Build next belief and proba
                // auto next_belief__eta = transition_function(pomdp, this->getptr(), action, observation, t);

                auto next_belief = std::dynamic_pointer_cast<Belief>(belief); //std::dynamic_pointer_cast<Belief>(std::get<0>(next_belief__eta));

                // Store the probability of next belief
                this->belief_probability[action][observation] = probability;//std::get<1>(next_belief__eta);

                std::shared_ptr<BeliefStateGraph> node_ptr;
                std::shared_ptr<BeliefInterface> new_address_belief = this->exist(next_belief);
                if(new_address_belief == nullptr)
                {
                    // Create a successor node
                    node_ptr = std::make_shared<BeliefStateGraph>(std::static_pointer_cast<BeliefStateGraph>(this->getptr()), next_belief);

                    // Add the belief in the space of beliefs
                    this->belief_space->emplace(next_belief, node_ptr);
                }else
                {
                    // Get the successor node
                    node_ptr = this->belief_space->at(new_address_belief);
                }

                // Add the sucessor in the list of successors
                this->successors.emplace(action_observation, node_ptr);
                return std::static_pointer_cast<BeliefStateGraph>(this->getSuccessor(action_observation));
            }
            
        }
        else
        {
            // Return next belief without storing its value in the graph
            // auto next_belief__eta = transition_function(pomdp, this->getptr(), action, observation, t);
            auto next_belief = std::dynamic_pointer_cast<Belief>(belief); //std::dynamic_pointer_cast<Belief>(std::get<0>(next_belief__eta));
            return std::make_shared<BeliefStateGraph>(std::static_pointer_cast<BeliefStateGraph>(this->getptr()), next_belief);
        }
    }

    std::string BeliefStateGraph::str() const
    {
        std::ostringstream str_result;
        str_result <<"Belief State Graph - " <<this->getData()->str();
        return str_result.str();
    }

    std::shared_ptr<BeliefStateGraph> BeliefStateGraph::getptr()
    {
        return Graph<std::shared_ptr<BeliefInterface>, Pair<std::shared_ptr<Action>, std::shared_ptr<Observation>>>::downcasted_shared_from_this<BeliefStateGraph>();
    }

    template <class Archive>
    void BeliefStateGraph::serialize(Archive &archive, const unsigned int)
    {
        using boost::serialization::make_nvp;

        archive &boost::serialization::base_object<Graph<Belief, Pair<std::shared_ptr<Action>, std::shared_ptr<Observation>>>>(*this);
        archive &make_nvp("belief_space", belief_space);
        archive &make_nvp("belief_probability", belief_probability);
    }

    std::vector<std::shared_ptr<State>> BeliefStateGraph::getStates() const
    {
        return this->getData()->getStates();
    }

    size_t BeliefStateGraph::size() const
    {
        return this->getData()->size();
    }

    double BeliefStateGraph::getProbability(const std::shared_ptr<State> &state) const
    {
        return this->getData()->getProbability(state);
    }

    void BeliefStateGraph::setProbability(const std::shared_ptr<State> &state, double proba)
    {
        this->data_->setProbability(state, proba);
    }

    void BeliefStateGraph::addProbability(const std::shared_ptr<State> &state, double proba)
    {
        this->data_->addProbability(state, proba);
    }

    bool BeliefStateGraph::operator==(const std::shared_ptr<BeliefInterface> &other) const
    {
        return this->getData()->operator==(other);
    }
    double BeliefStateGraph::operator^(const std::shared_ptr<BeliefInterface> &other) const
    {
        return this->getData()->operator^(other);
    }
    double BeliefStateGraph::norm_1() const
    {
        return this->getData()->norm_1();
    }

    std::shared_ptr<VectorInterface<std::shared_ptr<State>, double>> BeliefStateGraph::getVectorInferface()
    {
        throw sdm::exception::NotImplementedException();
    }

    TypeState BeliefStateGraph::getTypeState() const
    {
        return TypeState::BELIEF_STATE;
    }


    void BeliefStateGraph::setDefaultValue(double value)
    {
        this->data_->setDefaultValue(value);
    }
    double BeliefStateGraph::getDefaultValue() const
    {
        return this->getData()->getDefaultValue();
    }

    std::shared_ptr<BeliefInterface> BeliefStateGraph::exist(const std::shared_ptr<BeliefInterface>&belief)
    {
        for(const auto element : *this->belief_space)
        {
            if(*std::dynamic_pointer_cast<Belief>(element.first) == *std::dynamic_pointer_cast<Belief>(belief))
            {
                return element.first;
            }
        }
        return nullptr;
    }


} // namespace sdm

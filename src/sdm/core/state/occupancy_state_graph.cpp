#include <sdm/core/state/occupancy_state_graph.hpp>
#include <sdm/exception.hpp>
#include <sdm/core/state/jhistory_tree.hpp>
namespace sdm
{
    OccupancyStateGraph::OccupancyStateGraph() : BeliefStateGraph(), action_space_map(std::make_shared<std::unordered_map<number, std::shared_ptr<Space>>>())
    {
    }

    OccupancyStateGraph::OccupancyStateGraph(const std::shared_ptr<BeliefInterface> &data) : BeliefStateGraph(data), action_space_map(std::make_shared<std::unordered_map<number, std::shared_ptr<Space>>>())
    {
    }

    // OccupancyStateGraph::OccupancyStateGraph(const std::vector<std::shared_ptr<State>> &list_states, const std::vector<double> &list_proba)
    //     : BeliefStateGraph(Belief(list_states, list_proba))
    // {
    // }

    OccupancyStateGraph::OccupancyStateGraph(const std::shared_ptr<OccupancyStateGraph> &predecessor, const std::shared_ptr<BeliefInterface> &belief)
        : BeliefStateGraph(predecessor, belief), action_space_map(std::make_shared<std::unordered_map<number, std::shared_ptr<Space>>>()){}

    std::shared_ptr<BeliefStateGraph> OccupancyStateGraph::next(const std::shared_ptr<BeliefInterface>&belief, double probability , const std::shared_ptr<POMDPInterface> &pomdp, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t, bool backup )
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
                auto next_belief = std::dynamic_pointer_cast<OccupancyState>(belief); //std::dynamic_pointer_cast<Belief>(std::get<0>(next_belief__eta));

                // Store the probability of next belief
                this->belief_probability[action][observation] = probability;//std::get<1>(next_belief__eta);

                std::shared_ptr<BeliefStateGraph> node_ptr;
                std::shared_ptr<BeliefInterface> new_address_belief = this->exist(next_belief);
                if(new_address_belief == nullptr)
                {
                    // Create a successor node
                    node_ptr = std::make_shared<OccupancyStateGraph>(std::static_pointer_cast<OccupancyStateGraph>(this->getptr()), next_belief);

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
            return std::make_shared<OccupancyStateGraph>(std::static_pointer_cast<OccupancyStateGraph>(this->getptr()), next_belief);
        }
    }

    std::string OccupancyStateGraph::str() const
    {
        std::ostringstream str_result;
        str_result <<"Occupancy State Graph - "<< this->getData()->str();
        return str_result.str();
    }


    std::shared_ptr<BeliefStateGraph> OccupancyStateGraph::getptr()
    {
        return Graph<std::shared_ptr<BeliefInterface>, Pair<std::shared_ptr<Action>, std::shared_ptr<Observation>>>::downcasted_shared_from_this<OccupancyStateGraph>();
    }

    TypeState OccupancyStateGraph::getTypeState() const
    {
        return TypeState::OCCUPANCY_STATE;
    }


    double OccupancyStateGraph::getProbability(const std::shared_ptr<State> &state) const
    {
        return BeliefStateGraph::getProbability(state);
    }

    void OccupancyStateGraph::setProbability(const std::shared_ptr<State> &state, double proba)
    {
        BeliefStateGraph::setProbability(state,proba);
    }

    void OccupancyStateGraph::addProbability(const std::shared_ptr<State> &state, double proba)
    {
        BeliefStateGraph::addProbability(state,proba);
    }

    double OccupancyStateGraph::getProbability(const std::shared_ptr<JointHistoryInterface> &joint_history, const std::shared_ptr<BeliefInterface> &belief) const
    {
        return this->data_->toOccupancyState()->getProbability(joint_history,belief);
    }

    void OccupancyStateGraph::setProbability(const std::shared_ptr<JointHistoryInterface> &joint_history, const std::shared_ptr<BeliefInterface> &belief, double proba)
    {
        this->data_->toOccupancyState()->setProbability(joint_history,belief,proba);
    }

    void OccupancyStateGraph::addProbability(const std::shared_ptr<JointHistoryInterface> &joint_history, const std::shared_ptr<BeliefInterface> &belief, double proba)
    {
        this->data_->toOccupancyState()->addProbability(joint_history,belief,proba);
    }

    /**
     * @brief Get the set of joint histories that are in the support of the occupancy state.
     * @return the possible joint hitories
     */
    const std::set<std::shared_ptr<JointHistoryInterface>> &OccupancyStateGraph::getJointHistories() const
    {
        return this->data_->toOccupancyState()->getJointHistories();
    }

    /**
     * @brief Get the set of states that are in the support of the occupancy state for a precise joint historiy.

        * @return the possible states
        */
    const std::set<std::shared_ptr<BeliefInterface>> &OccupancyStateGraph::getBeliefs() const
    {
        return this->data_->toOccupancyState()->getBeliefs();
    }

    /**
     * @brief Get the set of beliefs at a given joint history
     * 
     * @param jhistory 
     * @return const std::set<std::shared_ptr<BeliefInterface>>& 
     */
    const std::set<std::shared_ptr<BeliefInterface>> &OccupancyStateGraph::getBeliefsAt(const std::shared_ptr<JointHistoryInterface> &jhistory) const
    {
        return this->data_->toOccupancyState()->getBeliefsAt(jhistory);
    }

    /**
     * @brief Get the set of individual histories that are in the support of the occupancy state (for a given agent).
     * @param number the agent identifier
     */
    const std::set<std::shared_ptr<HistoryInterface>> &OccupancyStateGraph::getIndividualHistories(number ag_id) const
    {
        return this->data_->toOccupancyState()->getIndividualHistories(ag_id);
    }

    /**
     * @brief Get the set of individual histories that are in the support of the occupancy state (for all agents).
     */
    const std::vector<std::set<std::shared_ptr<HistoryInterface>>> &OccupancyStateGraph::getAllIndividualHistories() const
    {
        return this->data_->toOccupancyState()->getAllIndividualHistories();
    }

    /**
     * @brief Get the fully uncompressed occupancy state.
     */
    std::shared_ptr<OccupancyStateInterface> OccupancyStateGraph::getFullyUncompressedOccupancy() const
    {
        return this->data_->toOccupancyState()->getFullyUncompressedOccupancy();
    }

    /**
     * @brief Set the fully uncompressed occupancy state.
     */
    void OccupancyStateGraph::setFullyUncompressedOccupancy(const std::shared_ptr<OccupancyStateInterface> &occupancy)
    {
        this->data_->toOccupancyState()->setFullyUncompressedOccupancy(occupancy);
    }

    /**
     * @brief Get the one step uncompressed occupancy state. 
     */
    std::shared_ptr<OccupancyStateInterface> OccupancyStateGraph::getOneStepUncompressedOccupancy() const
    {
        return this->data_->toOccupancyState()->getOneStepUncompressedOccupancy();
    }

    /**
     * @brief Set the one step uncompressed occupancy state
     */
    void OccupancyStateGraph::setOneStepUncompressedOccupancy(const std::shared_ptr<OccupancyStateInterface> &occupancy)
    {
        this->data_->toOccupancyState()->setOneStepUncompressedOccupancy(occupancy);
    }

    /**
     * @brief Get the list of labels that corresponds to the list of ihistories.
     */
    Joint<std::shared_ptr<HistoryInterface>> OccupancyStateGraph::getJointLabels(const Joint<std::shared_ptr<HistoryInterface>> &list_ihistories) const
    {
        return this->data_->toOccupancyState()->getJointLabels(list_ihistories);
    }

    /**
     * @brief Get all the probability conditionning to a Joint History
     * 
     * @param std::shared_ptr<JointHistoryInterface> : Joint History
     */
    const double &OccupancyStateGraph::getProbabilityOverJointHistory(const std::shared_ptr<JointHistoryInterface> &joint_history) const
    {
        return this->data_->toOccupancyState()->getProbabilityOverJointHistory(joint_history);
    }

    /**
     * @brief Update the labels of multiple individual histories
     */
    void OccupancyStateGraph::updateJointLabels(const Joint<std::shared_ptr<HistoryInterface>> &list_ihistories, const Joint<std::shared_ptr<HistoryInterface>> &list_labels)
    {
        this->data_->toOccupancyState()->updateJointLabels(list_ihistories,list_labels);
    }

    /**
     * @brief Get the Compressed Joint History. 
     */
    std::shared_ptr<JointHistoryInterface> OccupancyStateGraph::getCompressedJointHistory(const std::shared_ptr<JointHistoryInterface> &joint_history) const
    {
        return this->data_->toOccupancyState()->getCompressedJointHistory(joint_history);
    }

    /**
     * @brief Get the probability over individual histories and precise agent
     * 
     * @param number Agent Id
     * @param typename jhistory_type::element_type::ihistory_type : Individual History
     */
    const double &OccupancyStateGraph::getProbabilityOverIndividualHistories(number agent, const std::shared_ptr<HistoryInterface> &ihistory) const
    {
        return this->data_->toOccupancyState()->getProbabilityOverIndividualHistories(agent,ihistory);
    }

    /**
     * @brief Compression for occupancy states based on belief state representation. 
     * To be in this representation, the type 'TState' have to be a derivation of the interface BeliefState.  
     * 
     * @return the compressed occupancy state 
     */
    std::shared_ptr<OccupancyStateInterface> OccupancyStateGraph::compress()
    {
        return this->data_->toOccupancyState()->compress();
    }

    void OccupancyStateGraph::finalize()
    {
        this->data_->toOccupancyState()->finalize();
    }

    std::shared_ptr<BeliefInterface> OccupancyStateGraph::exist(const std::shared_ptr<BeliefInterface>&current_belief)
    {
        for(const auto element : *this->belief_space)
        {
            if(current_belief->operator==(element.first))
            {
                return element.first;
            }
        }
        return nullptr;
    }

    std::shared_ptr<Space> OccupancyStateGraph::getActionSpaceAt(number t)
    {
        if(this->action_space_map->find(t) != this->action_space_map->end())
        {
            return this->action_space_map->at(t);
        }
        else
        {
            return nullptr;
        }
    }

    void OccupancyStateGraph::setActionSpaceAt(number t, std::shared_ptr<Space> action_space)
    {
        this->action_space_map->emplace(t, action_space);
    }

} // namespace sdm

#pragma once

// #include <sdm/types.hpp>
// #include <sdm/utils/struct/pair.hpp>
#include <sdm/utils/struct/recursive_map.hpp>
#include <sdm/utils/linear_algebra/mapped_vector.hpp>

// #include <sdm/core/joint.hpp>
// #include <sdm/core/state/history.hpp>
// #include <sdm/core/state/base/base_occupancy_state.hpp>
#include <sdm/core/state/occupancy_state_interface.hpp>

namespace sdm
{
  class PrivateOccupancyState;

  /**
   * @brief An occupancy state refers to the whole knowledge that a central planner can have access to take decisions.
   * 
   * @tparam TState refers to a number
   * @tparam TJointHistory_p refers to a joint histories
   */
  class OccupancyState :public MappedVector<std::shared_ptr<BaseState<Pair<std::shared_ptr<State>, std::shared_ptr<JointHistoryTreeInterface>>>>, double>,
                        //public std::enable_shared_from_this<OccupancyState>,
                        public OccupancyStateInterface
  {
  public:

    OccupancyState(double default_value);
    OccupancyState(number num_agents = 2, double default_value = 0.);
    OccupancyState(const OccupancyState &);

    /**
     * @brief Compression for occupancy states based on belief state representation. 
     * To be in this representation, the type 'TState' have to be a derivation of the interface BeliefState.  
     * 
     * @return the compressed occupancy state 
     */
    std::shared_ptr<OccupancyStateInterface> compress();

    void finalize();

    /**
     * @brief Get the fully uncompressed occupancy state.
     */
    std::shared_ptr<OccupancyStateInterface> getFullyUncompressedOccupancy() const;

    /**
     * @brief Set the fully uncompressed occupancy state.
     */
    void setFullyUncompressedOccupancy(const std::shared_ptr<OccupancyStateInterface> &);

    /**
     * @brief Get the one step uncompressed occupancy state. 
     */
    std::shared_ptr<OccupancyStateInterface> getOneStepUncompressedOccupancy() const;

    /**
     * @brief Set the one step uncompressed occupancy state
     */
    void setOneStepUncompressedOccupancy(const std::shared_ptr<OccupancyStateInterface> &);

    /**
     * @brief Get the list of labels that corresponds to the list of ihistories.
     */
    std::vector<std::shared_ptr<HistoryTreeInterface>> getJointLabels(const std::vector<std::shared_ptr<HistoryTreeInterface>> &) const;

    /**
     * @brief Update the labels of multiple individual histories
     */
    void updateJointLabels(const std::vector<std::shared_ptr<HistoryTreeInterface>> &, const std::vector<std::shared_ptr<HistoryTreeInterface>> &);

    /**
     * @brief Get the Compressed Joint History. 
     */
    std::shared_ptr<JointHistoryTreeInterface> getCompressedJointHistory(const std::shared_ptr<JointHistoryTreeInterface> &) const;

    std::shared_ptr<OccupancyState> getptr();

    /**
     * @brief Get the probability over individual histories and precise agent
     * 
     * @param number Agent Id
     * @param typename jhistory_type::element_type::ihistory_type : Individual History
     */
    const double &getProbabilityOverIndividualHistories(number, const std::shared_ptr<HistoryTreeInterface> &) const ;
    void setProbabilityOverIndividualHistories();

    // Provient de Base Occupancy State

    const std::set<std::shared_ptr<State>> &getStatesAt(const std::shared_ptr<JointHistoryTreeInterface> &) const;

    const std::set<std::shared_ptr<JointHistoryTreeInterface>> &getJointHistories() const;

    const std::set<std::shared_ptr<HistoryTreeInterface>> &getIndividualHistories(number ag_id) const;

    const std::vector<std::set<std::shared_ptr<HistoryTreeInterface>>> &getAllIndividualHistories() const;

    const double &getProbabilityOverJointHistory(const std::shared_ptr<JointHistoryTreeInterface>&) const;

    static double PRECISION;

    std::shared_ptr<State> getHiddenState(const std::shared_ptr<BaseState<Pair<std::shared_ptr<State>, std::shared_ptr<JointHistoryTreeInterface>>>> &) const;
    std::shared_ptr<JointHistoryTreeInterface> getHistory(const std::shared_ptr<BaseState<Pair<std::shared_ptr<State>, std::shared_ptr<JointHistoryTreeInterface>>>> &) const;

    // Pour le moment pas besoin car il faut changer l'interface de Belief Interface ! Donc après autorisation de david

    void setProbability(const std::shared_ptr<State> &, const std::shared_ptr<JointHistoryTreeInterface> &, double);
    void setProbability(const std::shared_ptr<State> &, double);
    void addProbability(const std::shared_ptr<State> &, const std::shared_ptr<JointHistoryTreeInterface> &, double);
    void addProbability(const std::shared_ptr<State> &, double);
    double getProbability(const std::shared_ptr<State> &, const std::shared_ptr<JointHistoryTreeInterface> &) const;
    double getProbability(const std::shared_ptr<State> &) const;


    std::string str() const;
    std::string str_hyperplan() const;
    TypeState getTypeState() const;
    size_t size() const ;
    std::vector<std::shared_ptr<State>> getStates() const;

    bool operator==(const std::shared_ptr<BeliefInterface> &other) const;

  protected:
    /** @brief This representation of occupancy states consists of private occupancy states for each agent*/
    Joint<RecursiveMap<std::shared_ptr<HistoryTreeInterface>, std::shared_ptr<PrivateOccupancyState>>> tuple_of_maps_from_histories_to_private_occupancy_states_;

    /** @brief Keep in memory the uncompressed occupancy states */
    std::shared_ptr<OccupancyStateInterface> fully_uncompressed_occupancy_state, one_step_left_compressed_occupancy_state, compressed_occupancy_state;

    /** @brief Keep relations between all private ihistories and labels */
    Joint<RecursiveMap<std::shared_ptr<HistoryTreeInterface>, std::shared_ptr<HistoryTreeInterface>>> private_ihistory_map_;

    /** @brief */
    Joint<RecursiveMap<std::shared_ptr<HistoryTreeInterface>, std::shared_ptr<HistoryTreeInterface>>> map_label_to_pointer;

    /** @brief Keep relation between list of individual histories and joint histories */
    RecursiveMap<std::vector<std::shared_ptr<HistoryTreeInterface>>, std::shared_ptr<JointHistoryTreeInterface>> jhistory_map_;

    /** @brief probability of a private history space for a precise agent */
    std::unordered_map<number, std::unordered_map<std::shared_ptr<HistoryTreeInterface>, double>> probability_ihistories;


    bool areIndividualHistoryLPE(const std::shared_ptr<HistoryTreeInterface> &, const std::shared_ptr<HistoryTreeInterface> &, number);

    /**
     * @brief Get the private occupancy state structure. This structure consists of a set of private occupancy state for each agent. 
     */
    const Joint<RecursiveMap<std::shared_ptr<HistoryTreeInterface>, std::shared_ptr<PrivateOccupancyState>>> &getPrivateOccupancyStates() const;

    /**
     * @brief Get a specific private occupancy state. The private occupancy state return is for a specific agent and a specific history of this agent. 
     * 
     * @param agent_id the agent id
     * @param ihistory the individual history
     * @return the corresponding private occupancy state
     */
    const std::shared_ptr<PrivateOccupancyState> &getPrivateOccupancyState(const number &agent_id, const std::shared_ptr<HistoryTreeInterface> &ihistory) const;

    /**
     * @brief Set the one step uncompressed occupancy state
     */
    void setCompressedOccupancy(const std::shared_ptr<OccupancyStateInterface> &);

    /**
     * @brief Get the one step uncompressed occupancy state. 
     */
    std::shared_ptr<OccupancyStateInterface> getCompressedOccupancy() const;

    /**
     * @brief Get the label that corresponds to the ihistory.
     */
    std::shared_ptr<HistoryTreeInterface> getLabel(const std::shared_ptr<HistoryTreeInterface> &ihistory, number agent_id) const;

    /**
     * @brief Update the label of a specific individual history
     */
    void updateLabel(number, const std::shared_ptr<HistoryTreeInterface> &, const std::shared_ptr<HistoryTreeInterface> &);

    //Provient de Base Occupancy State

    /**
     * @brief Create the State associated with a precise joint history tree
     * 
     */
    void setStatesAt();

    void setStates();


    /**
     * @brief Set the Joint Histories  of the Occupancy State
     * 
     */
    void setJointHistories();

    /**
     * @brief Get all Joint History associated with a precise agent and an individual history
     * 
     * @param number :Agent Id
     * @param typename jhistory_type::element_type::ihistory_type : Individual History
     */
    const std::set<std::shared_ptr<JointHistoryTreeInterface>> &getJointHistoryOverIndividualHistories(number, const std::shared_ptr<HistoryTreeInterface>&) const;
    void setJointHistoryOverIndividualHistories();

    void setProbabilityOverJointHistory();

    void setAllIndividualHistories();

    /**
     * @brief the number of agents 
     */
    number num_agents_ = 2;

    /**
     * @brief space of all reachable states, those in the support of the occupancy state
     * @comment: Should not be used since there are to much possible wrt each joint history, belief states whould have been a better choice.
     */
    std::vector<std::shared_ptr<State>> list_states;

    /**
     * @brief space of joint histories of all agents
     */
    std::set<std::shared_ptr<JointHistoryTreeInterface>> list_jhistories;

    /**
     * @brief space of joint history and state of all agents
     */
    RecursiveMap<std::shared_ptr<JointHistoryTreeInterface>, std::set<std::shared_ptr<State>>> list_jhistory_states;

    /**
     * @brief tuple of private history spaces, one private history space per agent
     */
    std::vector<std::set<std::shared_ptr<HistoryTreeInterface>>> all_list_ihistories;

    std::unordered_map<number, std::unordered_map<std::shared_ptr<HistoryTreeInterface>, std::set<std::shared_ptr<JointHistoryTreeInterface>>>> ihistories_to_jhistory;

    RecursiveMap<std::shared_ptr<JointHistoryTreeInterface>, double> probability_jhistories;
  };
} // namespace sdm
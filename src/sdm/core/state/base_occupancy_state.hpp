// #pragma once
// #include <sdm/types.hpp>
// #include <sdm/core/joint.hpp>
// #include <sdm/core/state/state.hpp>
// #include <sdm/core/state/belief_state.hpp>
// #include <sdm/core/state/interface/occupancy_state_interface.hpp>
// #include <sdm/core/state/interface/history_interface.hpp>
// #include <sdm/core/state/interface/joint_history_interface.hpp>
// #include <sdm/core/state/interface/belief_interface.hpp>
// #include <sdm/utils/struct/recursive_map.hpp>
// #include <sdm/utils/linear_algebra/mapped_vector.hpp>

// namespace sdm
// {
//     class PrivateOccupancyState;

//     /**
//      * @brief An occupancy state refers to the whole knowledge that a central planner can have access to take decisions.
//      * 
//      * @tparam TState refers to a number
//      * @tparam TJointHistory_p refers to a joint histories
//      */
//     class BaseOccupancyState : public OccupancyStateInterface,
//                                public Belief
//     {
//     public:
//         static double PRECISION;

//         OccupancyState();
//         OccupancyState(number num_agents);
//         OccupancyState(const OccupancyState &copy);

//         /**
//          * @brief Get the probability to be in a pair (History / Belief) 
//          * 
//          * @param pair_history_belief the address of the history / belief pair
//          * @return double the probability
//          */
//         double getProbability(const std::shared_ptr<State> &pair_history_belief) const;

//         /**
//          * @brief Get the probability to be in a pair (History / Belief) 
//          * 
//          * @param joint_history the history
//          * @param belief the belief
//          * @return double the probability
//          */
//         double getProbability(const std::shared_ptr<State> &joint_history, const std::shared_ptr<State> &belief) const;

//         /**
//          * @brief Set the probability to be in a pair (History / Belief) 
//          * 
//          * @param pair_history_belief the address of the history / belief pair
//          * @param proba the probability 
//          */
//         void setProbability(const std::shared_ptr<State> &pair_history_belief, double proba);

//         /**
//          * @brief Set the probability to be in a pair (History / Belief) 
//          * 
//          * @param joint_history the history
//          * @param belief the belief
//          * @param proba the probability
//          */
//         void setProbability(const std::shared_ptr<State> &joint_history, const std::shared_ptr<State> &belief, double proba);

//         /**
//          * @brief Add the probability to be in a pair (History / Belief) 
//          * 
//          * @param pair_history_belief the address of the history / belief pair
//          * @param proba the probability 
//          */
//         void addProbability(const std::shared_ptr<State> &pair_history_belief, double proba);

//         /**
//          * @brief Add the probability to be in a pair (History / Belief) 
//          * 
//          * @param joint_history the history
//          * @param belief the belief
//          * @param proba the probability
//          */
//         void addProbability(const std::shared_ptr<State> &joint_history, const std::shared_ptr<State> &belief, double proba);

//         /**
//          * @brief Get the set of joint histories that are in the support of the occupancy state.
//          * @return the possible joint hitories
//          */
//         const std::set<std::shared_ptr<JointHistoryInterface>> &getJointHistories() const;

//         /**
//          * @brief Get the set of states that are in the support of the occupancy state for a precise joint historiy.

//          * @return the possible states
//          */
//         const std::set<std::shared_ptr<BeliefInterface>> &getBeliefs() const;

//         /**
//          * @brief Get the set of states that are in the support of the occupancy state for a precise joint historiy.

//          * @return the possible states
//          */
//         const std::set<std::shared_ptr<BeliefInterface>> &getBeliefsAt(const std::shared_ptr<JointHistoryInterface> &jhistory) const;

//         /**
//          * @brief Get the set of individual histories that are in the support of the occupancy state (for a given agent).
//          * @param number the agent identifier
//          */
//         const std::set<std::shared_ptr<HistoryInterface>> &getIndividualHistories(number ag_id) const;

//         /**
//          * @brief Get the set of individual histories that are in the support of the occupancy state (for all agents).
//          */
//         const std::vector<std::set<std::shared_ptr<HistoryInterface>>> &getAllIndividualHistories() const;

//         /**
//          * @brief Allow the use of accessors in O(1). `finalize()` must be called at the end of the occupancy state creation.
//          */
//         void finalize();

//         /**
//          * @brief Get all the probability conditionning to a Joint History
//          * 
//          * @param std::shared_ptr<JointHistoryInterface> : Joint History
//          */
//         const double &getProbabilityOverJointHistory(const std::shared_ptr<JointHistoryInterface> &) const;

//         /**
//          * @brief Get the probability over individual histories and precise agent
//          * 
//          * @param number Agent Id
//          * @param typename jhistory_type::element_type::ihistory_type : Individual History
//          */
//         const double &getProbabilityOverIndividualHistories(number agent_id, const std::shared_ptr<HistoryInterface> &indiv_history) const;

//         /**
//          * @brief Get the type of state (i.e. an occupancy state)
//          * 
//          * @return return the occupancy state type
//          */
//         TypeState getTypeState() const;

//         std::shared_ptr<OccupancyState> getptr();

//         std::string str() const;

//         double operator^(const std::shared_ptr<BeliefInterface> &other) const;
//         bool operator==(const OccupancyState &other) const;
//         bool operator==(const std::shared_ptr<BeliefInterface> &other) const;

//         /**
//          * @brief Get the action space in this occupancy state.
//          * 
//          * @param t the timestep
//          * @return the action space 
//          */
//         std::shared_ptr<Space> getActionSpaceAt(number t);

//         /**
//          * @brief Set the action space that correspond to the occupancy state
//          * 
//          * @param t the timestep
//          * @param action_space the action space
//          */
//         void setActionSpaceAt(number t, std::shared_ptr<Space> action_space);

//     protected:
//         void setupIndividualHistories();
//         void setupBeliefsAndHistories();
//         void setProbabilityOverJointHistory();
//         void setProbabilityOverIndividualHistories();
//         void setup();

//         std::shared_ptr<JointHistoryBeliefPair> getPairPointer(const std::shared_ptr<JointHistoryInterface> &joint_history, const std::shared_ptr<State> &belief) const;
//         std::shared_ptr<JointHistoryStatePair> getPairHistStatePointer(const std::shared_ptr<JointHistoryInterface> &joint_history, const std::shared_ptr<State> &belief) const;


//         /** @brief The number of agents */
//         number num_agents_ = 2;

//         /** @brief Keep relation between list of individual histories and joint histories */
//         RecursiveMap<Joint<std::shared_ptr<HistoryInterface>>, std::shared_ptr<JointHistoryInterface>> jhistory_map_;

//         /**
//          * @brief Space of all reachable states, those in the support of the occupancy state
//          * @comment: Should not be used since there are to much possible wrt each joint history, belief states whould have been a better choice.
//          */
//         std::set<std::shared_ptr<BeliefInterface>> list_beliefs_;

//         /**
//          * @brief space of joint histories
//          */
//         std::set<std::shared_ptr<JointHistoryInterface>> list_joint_histories_;

//         /**
//          * @brief tuple of private history spaces, one private history space per agent
//          */
//         std::vector<std::set<std::shared_ptr<HistoryInterface>>> all_list_ihistories_;

//         /**
//          * @brief mapping from joint history to belief
//          */
//         RecursiveMap<std::shared_ptr<JointHistoryInterface>, std::set<std::shared_ptr<BeliefInterface>>> map_joint_history_to_belief_;

//         /** @brief Map an individual history to a set containing all joint histories containing this ihistory */
//         std::unordered_map<number, std::unordered_map<std::shared_ptr<HistoryInterface>, std::set<std::shared_ptr<JointHistoryInterface>>>> ihistories_to_jhistory_;

//         static RecursiveMap<std::pair<std::shared_ptr<JointHistoryInterface>, std::shared_ptr<BeliefInterface>>, std::shared_ptr<JointHistoryBeliefPair>> map_pair_hist_belief_to_pointer_;
//         static RecursiveMap<std::pair<std::shared_ptr<JointHistoryInterface>, std::shared_ptr<State>>, std::shared_ptr<JointHistoryStatePair>> map_pair_hist_state_to_pointer_;

//         /** @brief Probability of a private history space for a precise agent */
//         std::unordered_map<number, std::unordered_map<std::shared_ptr<HistoryInterface>, double>> probability_ihistories;

//         /** @brief Probability joint histories */
//         RecursiveMap<std::shared_ptr<JointHistoryInterface>, double> probability_jhistories;

//         /** @brief Map from timestep to space */
//         std::shared_ptr<std::unordered_map<number, std::shared_ptr<Space>>> action_space_map;
//     };
// } // namespace sdm

// namespace std
// {

//     template <>
//     struct hash<sdm::BaseOccupancyState>
//     {
//         typedef sdm::BaseOccupancyState argument_type;
//         typedef std::size_t result_type;
//         inline result_type operator()(const argument_type &in) const
//         {
//             return std::hash<sdm::MappedVector<std::shared_ptr<sdm::State>>>()(in, sdm::BaseOccupancyState::PRECISION);
//         }
//     };
// }
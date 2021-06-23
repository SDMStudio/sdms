#pragma once
#include <sdm/utils/struct/recursive_map.hpp>
#include <sdm/utils/linear_algebra/mapped_vector.hpp>
#include <sdm/core/joint.hpp>
#include <sdm/core/state/belief_state.hpp>
#include <sdm/core/state/interface/occupancy_state_interface.hpp>
#include <sdm/core/state/interface/history_interface.hpp>
#include <sdm/core/state/interface/jhistory_interface.hpp>
#include <sdm/core/state/interface/belief_interface.hpp>

namespace sdm
{
    class PrivateOccupancyState;

    /**
     * @brief An occupancy state refers to the whole knowledge that a central planner can have access to take decisions.
     * 
     * @tparam TState refers to a number
     * @tparam TJointHistory_p refers to a joint histories
     */
    class OccupancyState : public OccupancyStateInterface,
                           public Belief
    {
    public:
        static double PRECISION;

        OccupancyState(double default_value);
        OccupancyState(number num_agents = 2, double default_value = 0.);
        OccupancyState(const OccupancyState &copy);

        /**
         * @brief Get the set of joint histories that are in the support of the occupancy state.
         * @return the possible joint hitories
         */
        const std::set<std::shared_ptr<JointHistoryInterface>> &getJointHistories() const;

        /**
         * @brief Get the set of states that are in the support of the occupancy state for a precise joint historiy.

         * @return the possible states
         */
        const std::shared_ptr<BeliefInterface> &getBeliefAt(const std::shared_ptr<JointHistoryInterface> &jhistory) const;

        /**
         * @brief Get the set of states that are in the support of the occupancy state for a precise joint historiy.

         * @return the possible states
         */
        // const std::set<std::shared_ptr<State>> &getStatesAt(const std::shared_ptr<JointHistoryInterface> &jhistory) const
        // {

        // }

        /**
         * @brief Get the set of individual histories that are in the support of the occupancy state (for a given agent).
         * @param number the agent identifier
         */
        const std::set<std::shared_ptr<HistoryInterface>> &getIndividualHistories(number ag_id) const;

        /**
         * @brief Get the set of individual histories that are in the support of the occupancy state (for all agents).
         */
        const std::vector<std::set<std::shared_ptr<HistoryInterface>>> &getAllIndividualHistories() const;

        /**
         * @brief Return the state of a precise occupancy state
         */
        std::shared_ptr<State> getHiddenState(const std::shared_ptr<BaseState<Pair<std::shared_ptr<State>, std::shared_ptr<JointHistoryInterface>>>> &) const;

        /**
         * @brief Return the history of a precise occupancy state
         */
        std::shared_ptr<JointHistoryInterface> getHistory(const std::shared_ptr<BaseState<Pair<std::shared_ptr<State>, std::shared_ptr<JointHistoryInterface>>>> &) const;

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
        std::vector<std::shared_ptr<HistoryInterface>> getJointLabels(const std::vector<std::shared_ptr<HistoryInterface>> &) const;

        /**
         * @brief Get all the probability conditionning to a Joint History
         * 
         * @param std::shared_ptr<JointHistoryInterface> : Joint History
         */
        const double &getProbabilityOverJointHistory(const std::shared_ptr<JointHistoryInterface> &) const;

        /**
         * @brief Update the labels of multiple individual histories
         */
        void updateJointLabels(const std::vector<std::shared_ptr<HistoryInterface>> &, const std::vector<std::shared_ptr<HistoryInterface>> &);

        /**
         * @brief Get the Compressed Joint History. 
         */
        std::shared_ptr<JointHistoryInterface> getCompressedJointHistory(const std::shared_ptr<JointHistoryInterface> &) const;

        /**
         * @brief Compression for occupancy states based on belief state representation. 
         * To be in this representation, the type 'TState' have to be a derivation of the interface BeliefState.  
         * 
         * @return the compressed occupancy state 
         */
        std::shared_ptr<OccupancyStateInterface> compress();

        TypeState getTypeState() const;

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
        const std::set<std::shared_ptr<JointHistoryInterface>> &getJointHistoryOverIndividualHistories(number, const std::shared_ptr<HistoryInterface> &) const;
        void setJointHistoryOverIndividualHistories();

        void setProbabilityOverJointHistory();

        void setAllIndividualHistories();

        /**
    protected:
        /**
         * @brief the number of agents 
         */
        number num_agents_ = 2;

        /** @brief This representation of occupancy states consists of private occupancy states for each agent */
        Joint<RecursiveMap<std::shared_ptr<HistoryInterface>, std::shared_ptr<PrivateOccupancyState>>> tuple_of_maps_from_histories_to_private_occupancy_states_;

        /** @brief Keep in memory the uncompressed occupancy states */
        std::shared_ptr<OccupancyStateInterface> fully_uncompressed_occupancy_state, one_step_left_compressed_occupancy_state, compressed_occupancy_state;

        /** @brief Keep relations between all private ihistories and labels */
        Joint<RecursiveMap<std::shared_ptr<HistoryInterface>, std::shared_ptr<HistoryInterface>>> private_ihistory_map_;

        /** @brief */
        Joint<RecursiveMap<std::shared_ptr<HistoryInterface>, std::shared_ptr<HistoryInterface>>> map_label_to_pointer;

        /** @brief Keep relation between list of individual histories and joint histories */
        RecursiveMap<std::vector<std::shared_ptr<HistoryInterface>>, std::shared_ptr<JointHistoryInterface>> jhistory_map_;

        /** @brief probability of a private history space for a precise agent */
        std::unordered_map<number, std::unordered_map<std::shared_ptr<HistoryInterface>, double>> probability_ihistories;

        /**
         * @brief space of all reachable states, those in the support of the occupancy state
         * @comment: Should not be used since there are to much possible wrt each joint history, belief states whould have been a better choice.
         */
        std::vector<std::shared_ptr<BeliefInterface>> list_beliefs_;

        /**
         * @brief space of joint histories
         */
        std::vector<std::shared_ptr<JointHistoryInterface>> list_joint_histories_;

        /**
         * @brief tuple of private history spaces, one private history space per agent
         */
        std::vector<std::set<std::shared_ptr<HistoryInterface>>> all_list_ihistories;

        /**
         * @brief mapping from joint history to belief
         */
        RecursiveMap<std::shared_ptr<JointHistoryInterface>, std::shared_ptr<BeliefInterface>> map_joint_history_to_belief_;

        std::unordered_map<number, std::unordered_map<std::shared_ptr<HistoryInterface>, std::set<std::shared_ptr<JointHistoryInterface>>>> ihistories_to_jhistory_;

        RecursiveMap<std::shared_ptr<JointHistoryInterface>, double> probability_jhistories_;
    };
} // namespace sdm
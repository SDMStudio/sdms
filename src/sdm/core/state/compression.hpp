// #pragma once

// #include <sdm/types.hpp>
// #include <sdm/core/joint.hpp>
// #include <sdm/utils/struct/recursive_map.hpp>
// #include <sdm/core/state/interface/history_interface.hpp>
// #include <sdm/core/state/interface/joint_history_interface.hpp>
// #include <sdm/core/state/interface/occupancy_state_interface.hpp>
// #include <sdm/core/state/private_occupancy_state.hpp>

// namespace sdm
// {
//     /**
//      * @brief 
//      */
//     class Compression
//     {
//     public:
//         /**
//          * @brief Check equivalence between two individual histories of a specific agent.
//          * 
//          * @param ihistory_1 a first individual history of agent i  
//          * @param ihistory_2 a second individual history of agent i
//          * @param agent_identifier the agent identifier
//          * @return true if individual histories are equivalent
//          * @return false otherwise
//          */
//         bool areIndividualHistoryLPE(const std::shared_ptr<HistoryInterface> &ihistory_1, const std::shared_ptr<HistoryInterface> &ihistory_2, number agent_identifier);

//         /**
//          * @brief Compression for occupancy states based on belief state representation. 
//          * To be in this representation, the type 'TState' have to be a derivation of the interface BeliefState.  
//          * 
//          * @return the compressed occupancy state 
//          */
//         std::shared_ptr<OccupancyStateInterface> compress(const std::shared_ptr<OccupancyStateInterface> &state_to_compress);

//         /**
//          * @brief Get the list of labels that corresponds to the list of ihistories.
//          */
//         Joint<std::shared_ptr<HistoryInterface>> getJointLabels(const Joint<std::shared_ptr<HistoryInterface>> &) const;

//         /**
//          * @brief Update the labels of multiple individual histories
//          */
//         void updateJointLabels(const Joint<std::shared_ptr<HistoryInterface>> &list_of_indiv_histories, const Joint<std::shared_ptr<HistoryInterface>> &list_of_labels);

//         /**
//          * @brief Get the Compressed Joint History. 
//          */
//         std::shared_ptr<JointHistoryInterface> getCompressedJointHistory(const std::shared_ptr<JointHistoryInterface> &list_of_indiv_histories) const;

//         void finalize();

//     protected:
//         /** @brief For each agent, keep relations between all individual histories to the address of the corresponding label */
//         Joint<RecursiveMap<std::shared_ptr<HistoryInterface>, std::shared_ptr<std::shared_ptr<HistoryInterface>>>> maps_ihistory_to_ptr_on_label;

//         /** @brief For each agent, keep relations between all labels to their address in the list of labels .*/
//         Joint<RecursiveMap<std::shared_ptr<HistoryInterface>, std::shared_ptr<std::shared_ptr<HistoryInterface>>>> maps_label_to_ptr_on_label;

//         /** @brief This representation of occupancy states consists of private occupancy states for each agent */
//         Joint<RecursiveMap<std::shared_ptr<HistoryInterface>, std::shared_ptr<PrivateOccupancyState>>> tuple_of_maps_from_histories_to_private_occupancy_states_;
//     };

// } // namespace sdm
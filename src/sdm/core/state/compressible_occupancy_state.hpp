// #pragma once

// namespace sdm
// {
//     class OccupancyState : public BaseOccupancyState
//     {
//     public:
//         /**
//          * @brief Compression for occupancy states based on belief state representation. 
//          * To be in this representation, the type 'TState' have to be a derivation of the interface BeliefState.  
//          * 
//          * @return the compressed occupancy state 
//          */
//         virtual std::shared_ptr<OccupancyStateInterface> compress() = 0;

//         /**
//          * @brief Get the Compressed Joint History. 
//          */
//         virtual std::shared_ptr<JointHistoryInterface> getCompressedJointHistory(const std::shared_ptr<JointHistoryInterface> &joint_hisotory) const = 0;

//         /**
//          * @brief Update the labels of multiple individual histories
//          */
//         virtual void updateJointLabels(const Joint<std::shared_ptr<HistoryInterface>> &ihistories, const Joint<std::shared_ptr<HistoryInterface>> &labels) = 0;

//         /**
//          * @brief Get all the probability conditionning to a Joint History
//          * 
//          * @param std::shared_ptr<JointHistoryInterface> : Joint History
//          */
//         virtual const double &getProbabilityOverJointHistory(const std::shared_ptr<JointHistoryInterface> &) const = 0;

//         /**
//          * @brief Get the probability over individual histories and precise agent
//          * 
//          * @param number Agent Id
//          * @param typename jhistory_type::element_type::ihistory_type : Individual History
//          */
//         virtual const double &getProbabilityOverIndividualHistories(number agent_id, const std::shared_ptr<HistoryInterface> &ihistory) const = 0;

//         /**
//          * @brief Get the fully uncompressed occupancy state.
//          */
//         virtual std::shared_ptr<OccupancyStateInterface> getFullyUncompressedOccupancy() const = 0;

//         /**
//          * @brief Set the fully uncompressed occupancy state.
//          */
//         virtual void setFullyUncompressedOccupancy(const std::shared_ptr<OccupancyStateInterface> &) = 0;

//         /**
//          * @brief Get the one step uncompressed occupancy state. 
//          */
//         virtual std::shared_ptr<OccupancyStateInterface> getOneStepUncompressedOccupancy() const = 0;

//         /**
//          * @brief Set the one step uncompressed occupancy state
//          */
//         virtual void setOneStepUncompressedOccupancy(const std::shared_ptr<OccupancyStateInterface> &) = 0;

//     protected:
//         /** @brief Compression operator */
//         std::shared_ptr<Compression> compression = nullptr;

//         /**
//          * @brief Get the list of labels that corresponds to the list of ihistories.
//          */
//         virtual Joint<std::shared_ptr<HistoryInterface>> getJointLabels(const Joint<std::shared_ptr<HistoryInterface>> &ihistories) const = 0;
//     };
// } // namespace sdm

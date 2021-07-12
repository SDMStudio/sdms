
    // // #############################################
    // // ###### MANIPULATE REPRESENTATION ############
    // // #############################################

    // const Joint<RecursiveMap<std::shared_ptr<HistoryInterface>, std::shared_ptr<PrivateOccupancyState>>> &OccupancyState::getPrivateOccupancyStates() const
    // {
    //     return this->tuple_of_maps_from_histories_to_private_occupancy_states_;
    // }

    // const std::shared_ptr<PrivateOccupancyState> &OccupancyState::getPrivateOccupancyState(const number &agent_id, const std::shared_ptr<HistoryInterface> &ihistory) const
    // {
    //     return this->tuple_of_maps_from_histories_to_private_occupancy_states_.at(agent_id).at(ihistory);
    // }

    // std::shared_ptr<OccupancyStateInterface> OccupancyState::getFullyUncompressedOccupancy() const
    // {
    //     return this->fully_uncompressed_occupancy_state;
    // }

    // void OccupancyState::setFullyUncompressedOccupancy(const std::shared_ptr<OccupancyStateInterface> &fully_uncompressed_ostate)
    // {
    //     this->fully_uncompressed_occupancy_state = fully_uncompressed_ostate;
    // }

    // std::shared_ptr<OccupancyStateInterface> OccupancyState::getOneStepUncompressedOccupancy() const
    // {
    //     return this->one_step_left_compressed_occupancy_state;
    // }

    // void OccupancyState::setOneStepUncompressedOccupancy(const std::shared_ptr<OccupancyStateInterface> &one_step_uncompress_ostate)
    // {
    //     this->one_step_left_compressed_occupancy_state = one_step_uncompress_ostate;
    //     std::static_pointer_cast<OccupancyState>(one_step_uncompress_ostate)->setCompressedOccupancy(this->getptr());
    // }

    // std::shared_ptr<OccupancyStateInterface> OccupancyState::getCompressedOccupancy() const
    // {
    //     return this->compressed_occupancy_state;
    // }

    // void OccupancyState::setCompressedOccupancy(const std::shared_ptr<OccupancyStateInterface> &compress_ostate)
    // {
    //     this->compressed_occupancy_state = compress_ostate;
    // }
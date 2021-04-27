#include <sdm/core/state/occupancy_state.hpp>
#include <sdm/core/state/private_occupancy_state.hpp>

namespace sdm
{

    template <typename TState, typename TJointHistory_p>
    OccupancyState<TState, TJointHistory_p>::OccupancyState() : OccupancyState<TState, TJointHistory_p>(0, 0)
    {
    }

    template <typename TState, typename TJointHistory_p>
    OccupancyState<TState, TJointHistory_p>::OccupancyState(double default_value) : OccupancyState<TState, TJointHistory_p>(0, default_value)
    {
    }

    template <typename TState, typename TJointHistory_p>
    OccupancyState<TState, TJointHistory_p>::OccupancyState(std::size_t size, double default_value)
        : BaseOccupancyState<TState, TJointHistory_p>(size, default_value)
    {
        // Build the private occupancy map
        for (number agent_id = 0; agent_id < this->num_agents; agent_id++)
        {
            this->tuple_of_maps_from_histories_to_private_occupancy_states_.push_back({});
            this->private_ihistory_map_.push_back({});
        }
    }

    template <typename TState, typename TJointHistory_p>
    OccupancyState<TState, TJointHistory_p>::OccupancyState(const OccupancyState &occupancy_state)
        : BaseOccupancyState<TState, TJointHistory_p>(occupancy_state),
          tuple_of_maps_from_histories_to_private_occupancy_states_(occupancy_state.getPrivateOccupancyStates()),
          fully_uncompressed_occupancy_state(occupancy_state.getFullyUncompressedOccupancy()),
          one_step_left_compressed_occupancy_state(occupancy_state.getOneStepUncompressedOccupancy()),
          private_ihistory_map_(occupancy_state.private_ihistory_map_)
    {

    }

    template <typename TState, typename TJointHistory_p>
    const Joint<RecursiveMap<typename OccupancyState<TState, TJointHistory_p>::ihistory_type, std::shared_ptr<PrivateOccupancyState<TState, TJointHistory_p>>>> &OccupancyState<TState, TJointHistory_p>::getPrivateOccupancyStates() const
    {
        return this->tuple_of_maps_from_histories_to_private_occupancy_states_;
    }

    template <typename TState, typename TJointHistory_p>
    const std::shared_ptr<PrivateOccupancyState<TState, TJointHistory_p>> &OccupancyState<TState, TJointHistory_p>::getPrivateOccupancyState(const number &agent_id, const ihistory_type &ihistory) const
    {
        return this->tuple_of_maps_from_histories_to_private_occupancy_states_.at(agent_id).at(ihistory);
    }

    template <typename TState, typename TJointHistory_p>
    bool OccupancyState<TState, TJointHistory_p>::areIndividualHistoryLPE(const typename TJointHistory_p::element_type::ihistory_type &ihistory_1, const typename TJointHistory_p::element_type::ihistory_type &ihistory_2, number agent_identifier)
    {
        return this->getPrivateOccupancyState(agent_identifier, ihistory_1) == this->getPrivateOccupancyState(agent_identifier, ihistory_2);
    }

    template <typename TState, typename TJointHistory_p>
    std::shared_ptr<OccupancyState<TState, TJointHistory_p>> OccupancyState<TState, TJointHistory_p>::getFullyUncompressedOccupancy() const
    {
        return this->fully_uncompressed_occupancy_state;
    }

    template <typename TState, typename TJointHistory_p>
    void OccupancyState<TState, TJointHistory_p>::setFullyUncompressedOccupancy(const std::shared_ptr<OccupancyState<TState, TJointHistory_p>> &fully_uncompressed_ostate)
    {
        this->fully_uncompressed_occupancy_state = fully_uncompressed_ostate;
    }

    template <typename TState, typename TJointHistory_p>
    std::vector<typename OccupancyState<TState, TJointHistory_p>::ihistory_type> OccupancyState<TState, TJointHistory_p>::getJointLabels(const std::vector<typename OccupancyState<TState, TJointHistory_p>::ihistory_type> &list_ihistories) const
    {
        std::vector<ihistory_type> new_list_ihistories;
        for (int agent = 0; agent < this->num_agents; ++agent)
        {
            if (this->private_ihistory_map_.at(agent).find(list_ihistories.at(agent)) == this->private_ihistory_map_.at(agent).end())
            {
                // if the ihistory was never compressed
                new_list_ihistories.push_back(list_ihistories.at(agent));
            }
            else
            {
                // if the ihistory was compressed
                new_list_ihistories.push_back(this->private_ihistory_map_.at(agent).at(list_ihistories.at(agent)));
            }
        }
        return new_list_ihistories;
    }

    template <typename TState, typename TJointHistory_p>
    std::shared_ptr<OccupancyState<TState, TJointHistory_p>> OccupancyState<TState, TJointHistory_p>::getOneStepUncompressedOccupancy() const
    {
        return this->one_step_left_compressed_occupancy_state;
    }

    template <typename TState, typename TJointHistory_p>
    void OccupancyState<TState, TJointHistory_p>::setOneStepUncompressedOccupancy(const std::shared_ptr<OccupancyState<TState, TJointHistory_p>> &one_step_uncompress_ostate)
    {
        this->one_step_left_compressed_occupancy_state = one_step_uncompress_ostate;
    }

    template <typename TState, typename TJointHistory_p>
    auto OccupancyState<TState, TJointHistory_p>::compress()
    {
        OccupancyState<TState, TJointHistory_p> current_compact_ostate;
        OccupancyState<TState, TJointHistory_p> previous_compact_ostate = *this;

        for (int agent_id = 0; agent_id < this->num_agents; ++agent_id)
        {
            auto support_set = this->getIndividualHistories(agent_id);
            auto support = tools::set2vector(support_set);
            // Sort support
            std::sort(support.begin(), support.end());

            for (auto iter_first = support.begin(); iter_first != support.end();)
            {
                auto ihistory_first = *iter_first;
                iter_first = support.erase(iter_first);
                for (auto iter_second = iter_first; iter_second != support.end();)
                {
                    auto ihistory_second = *iter_second;

                    if (this->areIndividualHistoryLPE(ihistory_first, ihistory_second, agent_id))
                    {
                        this->private_ihistory_map_[agent_id][ihistory_second] = ihistory_first; // Store label
                        iter_second = support.erase(iter_second);                                // Erase unecessary equivalent individual history

                        // Set probability of the compact occupancy state
                        for (const auto &pair_s_o_prob : *previous_compact_ostate.getPrivateOccupancyState(agent_id, ihistory_second))
                        {
                            current_compact_ostate.addProbabilityAt(pair_s_o_prob.first, pair_s_o_prob.second);
                        }
                    }
                    else
                    {
                        iter_second++;
                    }
                }
            }

            previous_compact_ostate = current_compact_ostate;
            current_compact_ostate.clear();
        }

        current_compact_ostate.setOneStepUncompressedOccupancy(this->getptr());
        current_compact_ostate.setFullyUncompressedOccupancy(this->getFullyUncompressedOccupancy());

        return current_compact_ostate;
    }

    template <typename TState, typename TJointHistory_p>
    void OccupancyState<TState, TJointHistory_p>::finalize()
    {
        BaseOccupancyState<TState, TJointHistory_p>::finalize();

        for (const auto &pair_state_jhist : *this)
        {
            auto jhist = this->getHistory(pair_state_jhist.first);
            auto proba = this->getProbability(pair_state_jhist.first);

            for (number agent_id = 0; agent_id < this->num_agents; agent_id++)
            {
                // Instanciation empty private occupancy state associated to ihitory and agent i if not exists
                if (this->tuple_of_maps_from_histories_to_private_occupancy_states_[agent_id].find(jhist->getIndividualHistory(agent_id)) == this->tuple_of_maps_from_histories_to_private_occupancy_states_[agent_id].end())
                {
                    this->tuple_of_maps_from_histories_to_private_occupancy_states_[agent_id].emplace(jhist->getIndividualHistory(agent_id), std::make_shared<PrivateOccupancyState<TState, TJointHistory_p>>(agent_id, this->default_value_));
                }
                // Set private occupancy measure
                this->tuple_of_maps_from_histories_to_private_occupancy_states_[agent_id][jhist->getIndividualHistory(agent_id)]->addProbabilityAt(pair_state_jhist.first, proba);
            }
        }
        for (number agent_id = 0; agent_id < this->num_agents; agent_id++)
        {
            for (const auto &pair_ihist_private_occupancy_state : this->tuple_of_maps_from_histories_to_private_occupancy_states_[agent_id])
            {
                pair_ihist_private_occupancy_state.second->finalize();
            }
        }

        // Build the private occupancy map
    }

    template <typename TState, typename TJointHistory_p>
    std::shared_ptr<OccupancyState<TState, TJointHistory_p>> OccupancyState<TState, TJointHistory_p>::getptr()
    {
        return std::static_pointer_cast<OccupancyState<TState, TJointHistory_p>>(this->shared_from_this());
    }

} // namespace sdm

namespace std
{
    template <typename S, typename V>
    struct hash<sdm::OccupancyState<S, V>>
    {
        typedef sdm::OccupancyState<S, V> argument_type;
        typedef std::size_t result_type;
        inline result_type operator()(const argument_type &in) const
        {
            return std::hash<sdm::BaseOccupancyState<S, V>>()(in);
        }
    };
}
#pragma once
#include <sdm/types.hpp>
#include <sdm/core/state/occupancy_state.hpp>

namespace sdm
{
    class PrivateOccupancyState;

    /**
     * @brief An occupancy state refers to the complete knowledge the central planner have access to take decisions.
     *
     * Occupancy states are firstly defined by Dibangoye, Amato, Buffet and Charpillet
     * in [Optimally Solving Dec-POMDPs as Continuous-State MDPs](https://hal.inria.fr/hal-01279444/document).
     * An occupancy state is defined as a posterior distribution over states and histories, given a complete information state
     * (i.e. \$\\xi_t (x_{t}, o_{t} ) = p(x_{t}, o_t \\mid i_{t})\$ ) .
     *
     */
    class CompressedOccupancyState : public OccupancyState
    {
    public:
        static double PRECISION;

        CompressedOccupancyState();
        CompressedOccupancyState(number num_agents);
        CompressedOccupancyState(const CompressedOccupancyState &copy);
        ~CompressedOccupancyState();

        /**
         * @brief Get the fully uncompressed occupancy state.
         */
        std::shared_ptr<OccupancyStateInterface> getFullyUncompressedOccupancy();

        /**
         * @brief Set the fully uncompressed occupancy state.
         */
        void setFullyUncompressedOccupancy(const std::shared_ptr<OccupancyStateInterface> &);

        /**
         * @brief Get the one step uncompressed occupancy state.
         */
        std::shared_ptr<OccupancyStateInterface> getOneStepUncompressedOccupancy();

        /**
         * @brief Set the one step uncompressed occupancy state
         */
        void setOneStepUncompressedOccupancy(const std::shared_ptr<OccupancyStateInterface> &);

        /**
         * @brief Get the compressed occupancy state
         */
        std::shared_ptr<OccupancyStateInterface> getCompressedOccupancy();

        /**
         * @brief Set the compressed occupancy state
         */
        void setCompressedOccupancy(const std::shared_ptr<OccupancyStateInterface> &compress_ostate);

        /**
         * @brief Get the label that corresponds to the ihistory.
         */
        std::shared_ptr<HistoryInterface> getLabel(const std::shared_ptr<HistoryInterface> &ihistory, number agent_id) const;

        /**
         * @brief Get the list of labels that corresponds to the list of ihistories.
         */
        Joint<std::shared_ptr<HistoryInterface>> getJointLabels(const Joint<std::shared_ptr<HistoryInterface>> &) const;

        /**
         * @brief Get the probability over individual histories and precise agent
         *
         * @param number Agent Id
         * @param typename jhistory_type::element_type::ihistory_type : Individual History
         */
        double getProbabilityOverIndividualHistories(number, const std::shared_ptr<HistoryInterface> &) const;

        /**
         * @brief Update the label of a specific individual history
         */
        void updateLabel(number, const std::shared_ptr<HistoryInterface> &, const std::shared_ptr<HistoryInterface> &);

        /**
         * @brief Update the labels of multiple individual histories
         */
        void updateJointLabels(const Joint<std::shared_ptr<HistoryInterface>> &, const Joint<std::shared_ptr<HistoryInterface>> &);

        /**
         * @brief Get the Compressed Joint History.
         */
        std::shared_ptr<JointHistoryInterface> getCompressedJointHistory(const std::shared_ptr<JointHistoryInterface> &) const;

        /**
         * @brief Check probabilistic equivalence
         *
         * @return true if histories are equivalent
         * @return false else
         */
        bool areIndividualHistoryLPE(const std::shared_ptr<HistoryInterface> &, const std::shared_ptr<HistoryInterface> &, number);

        /**
         * @brief Compression for occupancy states based on belief state representation.
         * To be in this representation, the type 'TState' have to be a derivation of the interface BeliefState.
         *
         * @return the compressed occupancy state
         */
        std::shared_ptr<OccupancyStateInterface> compress();

        /**
         * @brief Get the Private Occupancy States object
         *
         * @return const Joint<RecursiveMap<std::shared_ptr<HistoryInterface>, std::shared_ptr<PrivateOccupancyState>>>&
         */
        const Joint<RecursiveMap<std::shared_ptr<HistoryInterface>, std::shared_ptr<PrivateOccupancyState>>> &getPrivateOccupancyStates() const;

        /**
         * @brief Get the
         *
         * @param agent_id
         * @param ihistory
         * @return const std::shared_ptr<PrivateOccupancyState>&
         */
        const std::shared_ptr<PrivateOccupancyState> &getPrivateOccupancyState(const number &agent_id, const std::shared_ptr<HistoryInterface> &ihistory) const;

        // #############################################
        // ######### MANIPULATE TRANSITION ############
        // #############################################

        Pair<std::shared_ptr<State>, double> nextStateAndProba(const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t);

        void setupPrivateOccupancyStates();

    protected:
        /** @brief This representation of occupancy states consists of private occupancy states for each agent */
        Joint<RecursiveMap<std::shared_ptr<HistoryInterface>, std::shared_ptr<PrivateOccupancyState>>> tuple_of_maps_from_histories_to_private_occupancy_states_;

        Joint<RecursiveMap<std::shared_ptr<HistoryInterface>, double>> weight_of_private_occupancy_state_;

        /** @brief Keep in memory the uncompressed occupancy states */
        std::shared_ptr<OccupancyStateInterface> fully_uncompressed_occupancy_state, one_step_left_compressed_occupancy_state;

        std::weak_ptr<OccupancyStateInterface> compressed_occupancy_state;

        /** @brief Keep relations between all private ihistories and labels */
        Joint<RecursiveMap<std::shared_ptr<HistoryInterface>, std::shared_ptr<HistoryInterface>>> private_ihistory_map_;

        /** @brief probability of a private history space for a precise agent */
        std::unordered_map<number, std::unordered_map<std::shared_ptr<HistoryInterface>, double>> probability_ihistories;
    };
} // namespace sdm

namespace std
{
    template <>
    struct hash<sdm::CompressedOccupancyState>
    {
        typedef sdm::CompressedOccupancyState argument_type;
        typedef std::size_t result_type;
        inline result_type operator()(const argument_type &in, double precision) const
        {
            return std::hash<sdm::OccupancyState>()(in, sdm::CompressedOccupancyState::PRECISION);
        }

        inline result_type operator()(const argument_type &in) const
        {
            return std::hash<sdm::CompressedOccupancyState>()(in, sdm::OccupancyState::PRECISION);
        }
    };
}
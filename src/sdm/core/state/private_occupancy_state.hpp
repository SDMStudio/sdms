#pragma once

#include <sdm/core/state/occupancy_state.hpp>

namespace sdm
{

    /**
     * @brief A private occupancy state is an occupancy state (i.e. \$p(s_t, \\theta^{-i}_{t} \\mid \\iota_t, \\theta_t^i)\$ ).
     *
     */
    class PrivateOccupancyState : public OccupancyState
    {
    public:
        static double PRECISION_COMPRESSION;

        PrivateOccupancyState();
        PrivateOccupancyState(number num_agents);
        PrivateOccupancyState(number agent_id, number num_agents);
        PrivateOccupancyState(const PrivateOccupancyState &);
        PrivateOccupancyState(const OccupancyState &);

        /**
         * @brief Get the id of the agent that related to this occupancy state.
         *
         * @return number the agent id
         */
        number getAgentId() const;

        /**
         * @brief Get the partial joint history $\theta^{-i}$ corresponding to the joint history $\theta$
         *
         * @return the corresponding partial history
         */
        const std::vector<std::shared_ptr<HistoryInterface>> &getPartialJointHistory(const std::shared_ptr<JointHistoryInterface> &) const;

        /**
         * @brief Get the full joint history $\theta$ corresponding to the partial joint history $\theta^{-i}$
         *
         * @return the joint history
         */
        std::shared_ptr<JointHistoryInterface> getJointHistoryFromPartial(const std::vector<std::shared_ptr<HistoryInterface>> &) const;

        void finalize();
        void finalize(bool do_compression);

        /**
         * @brief Check the equivalence between two private occupancy states.
         *
         * @return true if private occupancy states are equivalent (given precision PRECISION_COMPRESSION ).
         * @return false if private occupancy states are not equivalent
         */
        bool check_equivalence(const PrivateOccupancyState &) const;

        std::string str() const;

    protected:
        /**
         * @brief Get the partial joint history $\theta^{-i}$ corresponding to the joint history $\theta$
         *
         * @return the corresponding partial history
         */
        std::vector<std::shared_ptr<HistoryInterface>> getPartialJointHistory(const std::vector<std::shared_ptr<HistoryInterface>> &) const;

        /** @brief The agent's identifier */
        number agent_id_;

        std::unordered_map<std::shared_ptr<JointHistoryInterface>, Joint<std::shared_ptr<HistoryInterface>>> map_jhist_to_partial;
        std::unordered_map<Joint<std::shared_ptr<HistoryInterface>>, std::shared_ptr<JointHistoryInterface>> map_partial_to_jhist;
    };
} // namespace sdm

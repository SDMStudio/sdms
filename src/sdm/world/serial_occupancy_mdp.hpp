#pragma once

#include <sdm/core/state/jhistory_tree.hpp>
#include <sdm/core/state/serial_occupancy_state.hpp>
#include <sdm/world/base/mmdp_interface.hpp>
#include <sdm/world/occupancy_mdp.hpp>

/**
 * @namespace  sdm
 * @brief Namespace grouping all tools required for sequential decision making.
 */
namespace sdm
{
        class SerialOccupancyMDP : public OccupancyMDP,
                                   public SerialProblemInterface
        {
        public:
                SerialOccupancyMDP();
                SerialOccupancyMDP(Config config);
                SerialOccupancyMDP(const std::shared_ptr<MPOMDPInterface> &dpomdp, Config config);
                SerialOccupancyMDP(const std::shared_ptr<SerialMPOMDPInterface> &dpomdp, number memory = -1, bool store_states = true, bool store_actions = true, int batch_size = 0);

                number getAgentId(number t) const;
                bool isLastAgent(number t) const;
                double getDiscount(number t) const;
                std::shared_ptr<SerialMMDPInterface> getUnderlyingSerialMMDP() const;
                std::shared_ptr<SerialMPOMDPInterface> getUnderlyingSerialMPOMDP() const;

                std::shared_ptr<Space> computeActionSpaceAt(const std::shared_ptr<State> &occupancy_state, number t = 0);
                std::shared_ptr<Action> computeRandomAction(const std::shared_ptr<OccupancyStateInterface> &ostate, number t);

                double getReward(const std::shared_ptr<State> &occupancy_state, const std::shared_ptr<Action> &decision_rule, number t);

        protected:
                /** @brief The underlying well defined MPOMDP */
                std::shared_ptr<SerialMPOMDPInterface> underlying_serial_mpomdp;
        };
} // namespace sdm

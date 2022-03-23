#pragma once

#include <sdm/core/state/jhistory_tree.hpp>
#include <sdm/core/state/serial_occupancy_state.hpp>
#include <sdm/core/state/occupancy_state_serial.hpp>
#include <sdm/world/base/mmdp_interface.hpp>
#include <sdm/world/occupancy_mdp.hpp>

/**
 * @namespace  sdm
 * @brief Namespace grouping all tools required for sequential decision making.
 */
namespace sdm
{
        template <typename TSerialOState>
        class BaseSerialOccupancyMDP : public BaseOccupancyMDP<TSerialOState>,
                                   public SerialProblemInterface
        {
        public:
                BaseSerialOccupancyMDP();
                BaseSerialOccupancyMDP(Config config);
                BaseSerialOccupancyMDP(const std::shared_ptr<MPOMDPInterface> &dpomdp, Config config);
                BaseSerialOccupancyMDP(const std::shared_ptr<MPOMDPInterface> &dpomdp, int memory = -1, bool store_states = true, bool store_actions = true, int batch_size = 0);

                number getHorizon() const;
                number getAgentId(number t) const;
                bool isLastAgent(number t) const;
                double getDiscount(number t) const;
                std::shared_ptr<MMDPInterface> getUnderlyingSerialMMDP() const;
                std::shared_ptr<MPOMDPInterface> getUnderlyingSerialMPOMDP() const;

                std::shared_ptr<Space> computeActionSpaceAt(const std::shared_ptr<State> &occupancy_state, number t = 0);
                std::shared_ptr<Action> computeRandomAction(const std::shared_ptr<OccupancyStateInterface> &ostate, number t);
                double getReward(const std::shared_ptr<State> &occupancy_state, const std::shared_ptr<Action> &decision_rule, number t);

                virtual std::shared_ptr<JointObservation> getDefaultObservation() const;

        protected:
                /** @brief The underlying well defined MPOMDP */
                std::shared_ptr<MPOMDPInterface> underlying_serial_mpomdp;

                /** @brief The empty observation */
                std::shared_ptr<JointObservation> empty_observation;

                /** @brief The horizon */
                number horizon;

                void setupEmptyObservation();
        };

        using SerialOccupancyMDP = BaseSerialOccupancyMDP<SerialOccupancyState>;
        using OccupancySerialMDP = BaseSerialOccupancyMDP<OccupancyStateSerial>;
} // namespace sdm

#include <sdm/world/serial_occupancy_mdp.tpp>
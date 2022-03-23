#pragma once

#include <sdm/types.hpp>
#include <sdm/core/action/action.hpp>
#include <sdm/core/state/interface/serial_interface.hpp>
#include <sdm/core/state/occupancy_state.hpp>
#include <sdm/core/joint.hpp>

namespace sdm
{

    /**
     * @brief The serial occupancy state class: a statistics for serial reformulations.
     *
     * Serial states are statistics used to solve serial games (an alternative reformulation of
     * simultaneous problems to drop the time complexity). A serial state is generally composed
     * of the classic state plus the list of actions chosen by the previous agents.
     *
     */
    class OccupancyStateSerial : public OccupancyState,
                                 public SerialInterface
    {
    public:
        OccupancyStateSerial();
        OccupancyStateSerial(number num_agents, number h, StateType stateType = COMPRESSED, Joint<std::shared_ptr<DecisionRule>> actions = {});
        OccupancyStateSerial(const OccupancyStateSerial &v);
        virtual ~OccupancyStateSerial();

        /**
         * @brief Get the agent ID corresponding to the current state.
         *
         * @return the current agent ID
         */
        number getCurrentAgentId() const;

        std::shared_ptr<Action> applyIndivDR(const std::shared_ptr<DecisionRule> &dr, const std::shared_ptr<JointHistoryInterface> &joint_history) const;

        /**
         * @brief Get the number of agents.
         *
         */
        number getNumAgents() const;

        std::shared_ptr<OccupancyState> make(number h);
        std::shared_ptr<OccupancyState> copy();

        double product(const std::shared_ptr<BetaVector> &beta, const std::shared_ptr<Action> &action);

        Pair<std::shared_ptr<State>, double> next(const std::shared_ptr<MDPInterface> &mdp, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t);
        double getReward(const std::shared_ptr<MDPInterface> &mdp, const std::shared_ptr<Action> &action, number t);

        size_t hash(double precision) const;
        bool operator==(const OccupancyStateSerial &other) const;
        bool isEqual(const OccupancyStateSerial &other, double precision) const;
        bool isEqual(const std::shared_ptr<State> &other, double precision) const;

        std::string str() const;

        Joint<std::shared_ptr<DecisionRule>> decision_rules = {};
        std::unordered_map<std::shared_ptr<JointHistoryInterface>, Joint<std::shared_ptr<Action>>> actions = {};
        std::shared_ptr<JointAction> getFullAction(const std::shared_ptr<MDPInterface> &mdp, Joint<std::shared_ptr<Action>> previous_actions, const std::shared_ptr<Action> &last_action, number t);

    protected:
        std::shared_ptr<DecisionRule> getFullDecisionRule(const std::shared_ptr<MDPInterface> &mdp, Joint<std::shared_ptr<DecisionRule>> previous_dr, const std::shared_ptr<DecisionRule> &last_dr, number t);
    };

} // namespace sdm

namespace std
{
    template <>
    struct hash<sdm::OccupancyStateSerial>
    {
        inline std::size_t operator()(const sdm::OccupancyStateSerial &in) const
        {
            return in.hash(sdm::OccupancyState::PRECISION);
        }
    };
}

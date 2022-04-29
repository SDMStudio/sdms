#pragma once

#include <sdm/core/state/state.hpp>
#include <sdm/core/action/action.hpp>
#include <sdm/core/reward/reward_model.hpp>
#include <sdm/utils/linear_algebra/mapped_matrix.hpp>

namespace sdm
{
    /**
     * @brief Model of the outcomes of the entire team of collaborative agents
     * involved in the system.
     *
     * In such a system (a cooperative game), the reward signal is common
     * to all agents.
     *
     */
    class CooperativeRewardModel : public TabularRewardModel
    {
    public:
        using data_t = MappedMatrix<std::shared_ptr<State>, std::shared_ptr<Action>, double>;

        CooperativeRewardModel();
        CooperativeRewardModel(const CooperativeRewardModel &copy);
        virtual ~CooperativeRewardModel();

        void initReward(number num_states, number num_actions);
        double getReward(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number agent_id, number t = 0) const;
        void setReward(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number agent_id, double reward, number t = 0);
        double getMinReward(number agent_id, number t = 0) const;
        double getMaxReward(number agent_id, number t = 0) const;

        friend std::ostream &operator<<(std::ostream &os, const CooperativeRewardModel &reward_fct)
        {
            os << "<cooperative-reward min=\"" << reward_fct.min << "\" max=\"" << reward_fct.max << "\"> " << std::endl;
            os << reward_fct.rewards_.str() << std::endl;
            os << "</cooperative-reward> " << std::endl;
            return os;
        }

    protected:
        data_t rewards_;

        double max = -std::numeric_limits<double>::max(), min = std::numeric_limits<double>::max();
    };

    /**
     * @brief Model of the outcomes of the agents involved in a non-cooperative system.
     *
     * In such a system (a non-cooperative game), the reward signal is specific to each agent.
     *
     */
    class CompetitiveRewardModel : public RewardModel
    {
    public:
        using data_t = std::unordered_map<std::shared_ptr<State>, std::unordered_map<std::shared_ptr<Action>, std::vector<double>>>;

        CompetitiveRewardModel();
        CompetitiveRewardModel(const CompetitiveRewardModel &copy);
        virtual ~CompetitiveRewardModel();

        void initReward(number num_states, number num_actions);
        double getReward(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number agent_id, number t = 0) const;
        void setReward(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number agent_id, double reward, number t = 0);
        void setReward(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, std::vector<double> rewards, number t = 0);
        double getMinReward(number agent_id, number t = 0) const;
        double getMaxReward(number agent_id, number t = 0) const;

        friend std::ostream &operator<<(std::ostream &os, const CompetitiveRewardModel &reward_fct)
        {
            os << "<competitive-reward> " << std::endl;
            // os << reward_fct.rewards_.str() << std::endl;
            os << "</competitive-reward> " << std::endl;
            return os;
        }

    protected:
        data_t rewards_;

        std::vector<double> max, min;
    };

}
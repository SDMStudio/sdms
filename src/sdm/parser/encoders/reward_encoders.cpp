#include <sdm/parser/encoders/reward_encoders.hpp>
#include <sdm/core/action/base_action.hpp>
#include <sdm/core/state/base_state.hpp>
#include <sdm/core/observation/base_observation.hpp>

namespace sdm
{
    namespace ast
    {
        tabular_reward_encoder::tabular_reward_encoder(const std::shared_ptr<DiscreteStateSpace> &state_space,
                                                       const std::shared_ptr<DiscreteItemSpace> &ag_space,
                                                       const std::shared_ptr<MultiDiscreteActionSpace> &action_space,
                                                       std::shared_ptr<CooperativeRewardModel> rewards) : boost::static_visitor<>()
        {
            this->state_space_ = state_space;
            this->ag_space_ = ag_space;
            this->action_space_ = action_space;
            this->rewards_ = rewards;
        }

        void tabular_reward_encoder::operator()(const reward_entry_1_t &r1)
        {
            double reward = r1.reward;
            state_encoder<StringState> s_encoder(this->state_space_);
            auto s_ptr = boost::apply_visitor(s_encoder, r1.state);

            joint_item_encoder<StringAction> ja_encoder(this->action_space_, this->ag_space_);
            auto joint_actions = ja_encoder.encode(r1.jaction);

            for (auto &state : s_ptr)
            {
                for (auto &joint_action : joint_actions)
                {
                    this->rewards_->setReward(state, joint_action, 0, reward);
                }
            }
        }

        void tabular_reward_encoder::operator()(const reward_entry_2_t &r2)
        {
            vector_encoder bl_encoder(this->state_space_->getNumItems());
            std::shared_ptr<VectorInterface<number, double>> vector_of_rewards = boost::apply_visitor(bl_encoder, r2.rewards);

            joint_item_encoder<StringAction> ja_encoder(this->action_space_, this->ag_space_);
            auto joint_actions = ja_encoder.encode(r2.jaction);

            for (auto &state : *this->state_space_)
            {
                for (auto &joint_action : joint_actions)
                {
                    this->rewards_->setReward(state, joint_action, 0,
                                              vector_of_rewards->getValueAt(this->state_space_->getItemIndex(state)));
                }
            }
        }

        tabular_rewards_encoder::tabular_rewards_encoder(const std::shared_ptr<DiscreteStateSpace> &state_space, const std::shared_ptr<DiscreteItemSpace> &agent_space, const std::shared_ptr<MultiDiscreteActionSpace> &action_space)
        {
            this->state_space_ = state_space;
            this->agent_space_ = agent_space;
            this->action_space_ = action_space;
        }

        std::shared_ptr<CooperativeRewardModel> tabular_rewards_encoder::encode(const reward_t &r)
        {
            std::shared_ptr<CooperativeRewardModel> rewards = std::make_shared<CooperativeRewardModel>();
            tabular_reward_encoder r_encoder(this->state_space_, this->agent_space_, this->action_space_, rewards);
            for (reward_entry_t const &rew : r)
            {
                boost::apply_visitor(r_encoder, rew);
            }
            return rewards;
        }

        multi_tabular_reward_encoder::multi_tabular_reward_encoder(const std::shared_ptr<DiscreteStateSpace> &state_space,
                                                                   const std::shared_ptr<DiscreteItemSpace> &ag_space,
                                                                   const std::shared_ptr<MultiDiscreteActionSpace> &action_space,
                                                                   std::shared_ptr<CompetitiveRewardModel> rewards) : boost::static_visitor<>()
        {
            this->state_space_ = state_space;
            this->ag_space_ = ag_space;
            this->action_space_ = action_space;
            this->rewards_ = rewards;
        }

        void multi_tabular_reward_encoder::operator()(const multi_reward_entry_1_t &r1)
        {
            double reward = r1.reward;

            state_encoder<StringItem> agent_encoder(this->ag_space_);
            auto list_agent = boost::apply_visitor(agent_encoder, r1.agent);

            state_encoder<StringState> s_encoder(this->state_space_);
            auto s_ptr = boost::apply_visitor(s_encoder, r1.state);

            joint_item_encoder<StringAction> ja_encoder(this->action_space_, this->ag_space_);
            auto joint_actions = ja_encoder.encode(r1.jaction);

            for (auto &state : s_ptr)
            {
                for (auto &joint_action : joint_actions)
                {
                    for (const auto &ag : list_agent)
                    {
                        this->rewards_->setReward(state, joint_action, this->ag_space_->getItemIndex(ag), reward);
                    }
                }
            }
        }

        void multi_tabular_reward_encoder::operator()(const multi_reward_entry_2_t &r2)
        {
            vector_encoder bl_encoder(this->ag_space_->getNumItems());
            std::shared_ptr<VectorInterface<number, double>> vector_of_rewards = boost::apply_visitor(bl_encoder, r2.rewards);
            std::vector<double> list_reward = {};
            for (int i = 0; i < this->ag_space_->getNumItems(); i++)
            {
                list_reward.push_back(vector_of_rewards->getValueAt(i));
            }

            state_encoder<StringState> s_encoder(this->state_space_);
            auto s_ptr = boost::apply_visitor(s_encoder, r2.state);

            joint_item_encoder<StringAction> ja_encoder(this->action_space_, this->ag_space_);
            auto joint_actions = ja_encoder.encode(r2.jaction);

            for (auto &state : s_ptr)
            {
                for (auto &joint_action : joint_actions)
                {
                    this->rewards_->setReward(state, joint_action, list_reward);
                }
            }
        }

        multi_tabular_rewards_encoder::multi_tabular_rewards_encoder(const std::shared_ptr<DiscreteStateSpace> &state_space, const std::shared_ptr<DiscreteItemSpace> &agent_space, const std::shared_ptr<MultiDiscreteActionSpace> &action_space)
        {
            this->state_space_ = state_space;
            this->agent_space_ = agent_space;
            this->action_space_ = action_space;
        }

        std::shared_ptr<CompetitiveRewardModel> multi_tabular_rewards_encoder::encode(const multi_reward_t &r)
        {
            std::shared_ptr<CompetitiveRewardModel> rewards = std::make_shared<CompetitiveRewardModel>();
            multi_tabular_reward_encoder r_encoder(this->state_space_, this->agent_space_, this->action_space_, rewards);
            for (multi_reward_entry_t const &rew : r)
            {
                boost::apply_visitor(r_encoder, rew);
            }
            return rewards;
        }

    } // namespace ast

} // namespace sdm

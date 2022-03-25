#pragma once

#include <sdm/parser/ast.hpp>

#include <sdm/core/item.hpp>
#include <sdm/core/space/discrete_space.hpp>
#include <sdm/core/space/multi_discrete_space.hpp>
#include <sdm/core/reward/reward_model.hpp>
#include <sdm/core/reward/tabular_reward.hpp>
#include <sdm/parser/encoders/item_encoders.hpp>
#include <sdm/parser/encoders/struct_encoders.hpp>
#include <sdm/parser/encoders/space_encoders.hpp>

namespace sdm
{
    namespace ast
    {

        struct tabular_reward_encoder : boost::static_visitor<>
        {
            std::shared_ptr<DiscreteSpace> state_space_;
            std::shared_ptr<DiscreteSpace> ag_space_;
            std::shared_ptr<MultiDiscreteSpace> action_space_;

            std::shared_ptr<CooperativeRewardModel> rewards_;

            tabular_reward_encoder(const std::shared_ptr<DiscreteSpace> &state_space,
                                   const std::shared_ptr<DiscreteSpace> &ag_space,
                                   const std::shared_ptr<MultiDiscreteSpace> &action_space,
                                   std::shared_ptr<CooperativeRewardModel> rewards);

            void operator()(const reward_entry_1_t &r1);
            void operator()(const reward_entry_2_t &r2);
        };

        struct tabular_rewards_encoder
        {
            std::shared_ptr<DiscreteSpace> state_space_;
            std::shared_ptr<MultiDiscreteSpace> action_space_;
            std::shared_ptr<DiscreteSpace> agent_space_;

            tabular_rewards_encoder(const std::shared_ptr<DiscreteSpace> &state_space, const std::shared_ptr<DiscreteSpace> &agent_space, const std::shared_ptr<MultiDiscreteSpace> &action_space);

            std::shared_ptr<CooperativeRewardModel> encode(const reward_t &r);
        };

        struct multi_tabular_reward_encoder : boost::static_visitor<>
        {
            std::shared_ptr<DiscreteSpace> state_space_;
            std::shared_ptr<DiscreteSpace> ag_space_;
            std::shared_ptr<MultiDiscreteSpace> action_space_;

            std::shared_ptr<CompetitiveRewardModel> rewards_;

            multi_tabular_reward_encoder(const std::shared_ptr<DiscreteSpace> &state_space,
                                         const std::shared_ptr<DiscreteSpace> &ag_space,
                                         const std::shared_ptr<MultiDiscreteSpace> &action_space,
                                         std::shared_ptr<CompetitiveRewardModel> rewards);

            void operator()(const multi_reward_entry_1_t &r1);
            void operator()(const multi_reward_entry_2_t &r2);
        };

        struct multi_tabular_rewards_encoder
        {
            std::shared_ptr<DiscreteSpace> state_space_;
            std::shared_ptr<MultiDiscreteSpace> action_space_;
            std::shared_ptr<DiscreteSpace> agent_space_;

            multi_tabular_rewards_encoder(const std::shared_ptr<DiscreteSpace> &state_space, const std::shared_ptr<DiscreteSpace> &agent_space, const std::shared_ptr<MultiDiscreteSpace> &action_space);

            std::shared_ptr<CompetitiveRewardModel> encode(const multi_reward_t &r);
        };
    }

}
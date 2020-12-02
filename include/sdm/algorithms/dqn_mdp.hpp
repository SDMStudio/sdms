#pragma once

#include <torch/torch.h>
#include <vector>

#include <sdm/types.hpp>
#include <sdm/public/algorithm.hpp>
#include <sdm/world/posg.hpp>
#include <sdm/utils/nn/mlpnet.hpp>
#include <sdm/utils/rl/eps_greedy.hpp>
#include <sdm/utils/rl/replay_memory.hpp>

//! \namespace  sdm
//! \brief Namespace grouping all tools required for sequential decision making.
namespace sdm
{
    class DQNMDP : public Algorithm
    {
    protected:
        //! \brief Policy model
        DQN policy_net_;

        //! \brief Target model
        DQN target_net_;

        //! \brief The optimizer
        torch::optim::Adam optimizer;

        //! \brief The replay memory
        ReplayMemory replay_memory_;

        //! \brief The eps-greedy exploration process
        EpsGreedy eps_greedy_;

        //! \brief The batch size
        number batch_size_;

        //! \brief target update frequency
        number target_update_ = 1;

        //! \brief The current number of steps done
        number step_done_ = 0;

        //! \brief The current number of steps done
        torch::Device device_;

        action select_action(const std::shared_ptr<POSG> &, state, std::uniform_int_distribution<int> &);
        void optimize_dqn(const std::shared_ptr<POSG> &, bool = false);

    public:
        DQNMDP(DQN, DQN, ReplayMemory, EpsGreedy, int, int, torch::Device = torch::kCPU);
        void to(torch::Device);
        void solve(const std::shared_ptr<POSG> &, horizon, double = 0.001, double = 1.0);
    };
} // namespace sdm
#pragma once

#include <sdm/utils/value_function/backup/qvalue_backup_interface.hpp>
#include <sdm/utils/nn/dqn.hpp>
#include <sdm/utils/rl/deep_experience_memory.hpp>
#include <sdm/core/space/multi_discrete_space.hpp>


namespace sdm
{

    class DqnBmdpBackup : public QValueBackupInterface
    {
    public:

        DqnBmdpBackup();
        DqnBmdpBackup(std::shared_ptr<ExperienceMemoryInterface> experience_memory, std::shared_ptr<DQN> policy_net, std::shared_ptr<DQN> target_net, double discount, number horizon, number batch_size, double lr, std::shared_ptr<Space> state_space, std::shared_ptr<Space> action_space);
        
        ~DqnBmdpBackup();

        double update();

        /**
         * @brief 
         * 
         * @param const std::shared_ptr<State>& state : current state
         * @param number t : time step
         * @return 
         */
        std::shared_ptr<Action> getGreedyAction(const std::shared_ptr<State> &state, number t);

        /**
         * @brief 
         * 
         * @param const std::shared_ptr<State>& state : current state
         * @param number t : time step
         * @return 
         */
        double getValueAt(const std::shared_ptr<State> &state, number t);

    protected:
        using sars_transition = ExperienceMemoryInterface::sars_transition;

        std::shared_ptr<DeepExperienceMemory> experience_memory_;
        std::shared_ptr<DQN> policy_net_;
        std::shared_ptr<DQN> target_net_;
        double discount_;
        number horizon_;
        number batch_size_;
        double lr_;
        std::shared_ptr<DiscreteSpace> state_space_;
        std::shared_ptr<MultiDiscreteSpace> action_space_;
        std::shared_ptr<torch::optim::Optimizer> optimizer_;

        std::tuple<torch::Tensor, torch::Tensor, torch::Tensor, torch::Tensor, torch::Tensor, torch::Tensor, torch::Tensor> constructBatch(std::vector<sars_transition> transitions);
        torch::Tensor getQValues(torch::Tensor t_batch, torch::Tensor b_batch, torch::Tensor u_batch);
        torch::Tensor getTargetQValues(torch::Tensor next_t_batch, torch::Tensor next_b_batch, torch::Tensor r_batch, torch::Tensor regular_t_batch);
    };
    

} // namespace sdm

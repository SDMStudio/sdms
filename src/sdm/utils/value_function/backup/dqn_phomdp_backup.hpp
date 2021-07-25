#pragma once

#include <sdm/utils/value_function/backup/qvalue_backup_interface.hpp>
#include <sdm/utils/nn/dqn.hpp>
#include <sdm/utils/rl/deep_experience_memory.hpp>
#include <sdm/core/space/multi_discrete_space.hpp>


namespace sdm
{

    class DqnPhomdpBackup : public QValueBackupInterface
    {
    public:

        DqnPhomdpBackup();
        DqnPhomdpBackup(std::shared_ptr<ExperienceMemoryInterface> experience_memory, std::shared_ptr<DQN> policy_net, std::shared_ptr<DQN> target_net, double discount, number horizon, number batch_size, double lr, std::shared_ptr<Space> state_space, std::shared_ptr<Space> observation_space, std::shared_ptr<Space> action_space);
        
        ~DqnPhomdpBackup();

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
        std::shared_ptr<MultiDiscreteSpace> observation_space_;
        std::shared_ptr<DiscreteSpace> observation_space_1_;
        std::shared_ptr<DiscreteSpace> observation_space_2_;
        std::shared_ptr<MultiDiscreteSpace> action_space_;
        std::shared_ptr<torch::optim::Optimizer> optimizer_;

        std::tuple<torch::Tensor, torch::Tensor, torch::Tensor, torch::Tensor, torch::Tensor, torch::Tensor, torch::Tensor, torch::Tensor, torch::Tensor, torch::Tensor, torch::Tensor, torch::Tensor> constructBatch(std::vector<sars_transition> transitions);
        torch::Tensor getQValues(torch::Tensor t_batch, torch::Tensor s_batch, torch::Tensor z1_batch, torch::Tensor z2_batch, torch::Tensor u_batch);
        torch::Tensor getTargetQValues(torch::Tensor next_t_batch, torch::Tensor next_s_batch, torch::Tensor next_z1_batch, torch::Tensor next_z2_batch, torch::Tensor next_u_batch, torch::Tensor r_batch, torch::Tensor regular_t_batch);
        std::shared_ptr<Action> applyDecisionRule(const std::shared_ptr<Observation> &observation, const std::shared_ptr<Action> &decision_rule, number t) const;
        
    };
    

} // namespace sdm

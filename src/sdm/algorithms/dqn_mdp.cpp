#include <sdm/algorithms/dqn_mdp.hpp>
#include <sdm/utils/rl/replay_memory.hpp>

namespace sdm
{

    DQNMDP::DQNMDP(DQN policy_net, DQN target_net, ReplayMemory replay_memo, EpsGreedy eps_greedy, int batch_size, int target_update, torch::Device device) : policy_net_(policy_net), target_net_(target_net), replay_memory_(replay_memo), eps_greedy_(eps_greedy), batch_size_(batch_size), target_update_(target_update), optimizer(policy_net->parameters()), device_(device)
    {
    }

    void DQNMDP::to(torch::Device device)
    {
        this->device_ = device;
        this->policy_net_->to(device);
        this->target_net_->to(device);
    }

    void DQNMDP::solve(const std::shared_ptr<POSG> &world, number episodes, double, double)
    {

        std::stringstream stream;
        torch::save(this->policy_net_, stream);
        torch::load(this->target_net_, stream);

        std::uniform_int_distribution<int> DISTRIBUTION_ACTIONS(0, world->getNumJActions() - 1);

        this->step_done_ = 0;
        for (number episode = 0; episode < episodes; episode++)
        {
            state x = world->init();
            float dis_cum_r = 0;
            for (number step = 0; step < world->getPlanningHorizon(); step++)
            {
                this->step_done_++;
                action a = select_action(world, x, DISTRIBUTION_ACTIONS);
                std::tuple<std::vector<double>, observation, state> r_w_y = world->getDynamicsGenerator(x, a);
                float r = std::get<0>(r_w_y)[0];
                state y = std::get<2>(r_w_y);
                sars_transition t = std::make_tuple(x, a, r, y);
                this->replay_memory_.push(t);
                dis_cum_r += pow(world->getDiscount(), step) * r;
                optimize_dqn(world);
                if ((episode * world->getPlanningHorizon() + step) % this->target_update_ == 0)
                {
                    std::stringstream stream;
                    torch::save(this->policy_net_, stream);
                    torch::load(this->target_net_, stream);
                }
                x = y;
            }
            std::cout << "episode " << episode << " | discounted cumulative reward " << dis_cum_r << std::endl;
        }

        std::vector<torch::Tensor> state_batch_vector;
        for (state x = 0; x < world->getNumStates(); x++)
        {
            torch::Tensor one_hot_x = torch::zeros(world->getNumStates());
            one_hot_x[x] = 1;
            state_batch_vector.push_back(one_hot_x);
        }

        torch::Tensor state_batch = torch::cat(state_batch_vector);
        state_batch = state_batch.reshape({world->getNumStates(), -1});
        std::cout << std::endl << "state_batch" << std::endl << state_batch << std::endl << std::endl;
        std::cout << "policy_net(state_batch)" << std::endl << this->policy_net_(state_batch) << std::endl << std::endl;
        std::cout << "policy_net(state_batch).argmax(1)" << std::endl << this->policy_net_(state_batch).argmax(1) << std::endl;
    }

    action DQNMDP::select_action(const std::shared_ptr<POSG> &world, state x, std::uniform_int_distribution<int> &action_distrib)
    {
        float sample = this->eps_greedy_.eps_distrib(sdm::common::global_urng());
        float epsilon = this->eps_greedy_.epsilon_end + (this->eps_greedy_.epsilon_start - this->eps_greedy_.epsilon_end) * exp(-1. * this->step_done_ / this->eps_greedy_.epsilon_decay);
        action a;
        if (sample > epsilon)
        {
            torch::NoGradGuard no_grad;
            torch::Tensor one_hot_x = torch::zeros(world->getNumStates());
            one_hot_x[x] = 1;
            a = torch::argmax(this->policy_net_(one_hot_x)).item<int>();
        }
        else
        {
            a = action_distrib(sdm::common::global_urng());
        }
        return a;
    }

    void DQNMDP::optimize_dqn(const std::shared_ptr<POSG> &world, bool print = false)
    {
        if (this->replay_memory_.size() < this->batch_size_)
        {
            return;
        }

        std::vector<sars_transition> transitions = this->replay_memory_.sample(this->batch_size_);

        std::vector<torch::Tensor> state_batch_vector;
        std::vector<long> action_batch_vector;
        std::vector<long> reward_batch_vector;
        std::vector<torch::Tensor> next_state_batch_vector;

        for (sars_transition t : transitions)
        {
            state x = std::get<0>(t);
            torch::Tensor one_hot_x = torch::zeros(world->getNumStates());
            one_hot_x[x] = 1;
            state_batch_vector.push_back(one_hot_x);
            action_batch_vector.push_back(std::get<1>(t));
            reward_batch_vector.push_back(std::get<2>(t));
            state y = std::get<3>(t);
            torch::Tensor one_hot_y = torch::zeros(world->getNumStates());
            one_hot_y[y] = 1;
            next_state_batch_vector.push_back(one_hot_y);
        }

        torch::Tensor state_batch = torch::cat(state_batch_vector); // idk if keeps the same device
        state_batch = state_batch.reshape({this->batch_size_, -1});
        torch::Tensor action_batch = torch::tensor(action_batch_vector);
        torch::Tensor reward_batch = torch::tensor(reward_batch_vector);
        torch::Tensor next_state_batch = torch::cat(next_state_batch_vector); // idk if keeps the same device
        next_state_batch = next_state_batch.reshape({this->batch_size_, -1});
        if (print)
        {
            std::cout << std::endl
                      << std::endl
                      << "state_batch" << std::endl
                      << state_batch << std::endl
                      << std::endl;
            std::cout << "action_batch" << std::endl
                      << action_batch << std::endl
                      << std::endl;
            std::cout << "reward_batch" << std::endl
                      << reward_batch << std::endl
                      << std::endl;
            std::cout << "next_state_batch" << std::endl
                      << next_state_batch << std::endl
                      << std::endl;
        }
        action_batch = action_batch.unsqueeze(1);
        torch::Tensor state_action_values = this->policy_net_(state_batch).gather(-1, action_batch);

        // Compute V(s_{t+1}) for all next states. Expected values of actions for non_final_next_states are computed based on the "older" target_net; selecting their best
        // reward with max(1)[0]. This is merged based on the mask, such that we'll have either the expected state value or 0 in case the state was final.
        torch::Tensor next_state_values = torch::zeros(this->batch_size_, this->device_);
        next_state_values = std::get<0>(this->target_net_(next_state_batch).max(1)).detach();

        // Compute the expected Q values
        torch::Tensor expected_state_action_values = (next_state_values * world->getDiscount()) + reward_batch;

        // Compute Huber loss
        torch::Tensor loss = at::smooth_l1_loss(state_action_values, expected_state_action_values.unsqueeze(1));
        if (print)
        {
            std::cout << "action_batch" << std::endl
                      << action_batch << std::endl
                      << std::endl;
            std::cout << "policy_net(state_batch)" << std::endl
                      << this->policy_net_(state_batch) << std::endl
                      << std::endl;
            std::cout << "state_action_values" << std::endl
                      << state_action_values << std::endl
                      << std::endl;
            std::cout << "target_net(next_state_batch)" << std::endl
                      << this->target_net_(next_state_batch) << std::endl
                      << std::endl;
            std::cout << "next_state_values" << std::endl
                      << next_state_values << std::endl
                      << std::endl;
            std::cout << "expected_state_action_values" << std::endl
                      << expected_state_action_values << std::endl
                      << std::endl;
            std::cout << "loss" << std::endl
                      << loss << std::endl
                      << std::endl;
        }

        // Optimize the model
        this->optimizer.zero_grad();
        loss.backward();
        for (auto param : this->policy_net_->parameters())
        {
            param.grad().data().clamp_(-1, 1); // Seems to be ok
        }
        this->optimizer.step();
    }

} // namespace sdm

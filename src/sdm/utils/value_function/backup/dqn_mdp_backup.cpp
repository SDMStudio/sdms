#include <sdm/utils/value_function/backup/dqn_mdp_backup.hpp>

namespace sdm
{
    DqnMdpBackup::DqnMdpBackup()
    {

    }

    DqnMdpBackup::DqnMdpBackup(
        std::shared_ptr<ExperienceMemoryInterface> experience_memory, 
        std::shared_ptr<DQN> policy_net, 
        std::shared_ptr<DQN> target_net, 
        double discount, 
        number horizon, 
        number batch_size,
        double lr,
        std::shared_ptr<Space> state_space, 
        std::shared_ptr<Space> action_space
    ) 
        :   experience_memory_(std::static_pointer_cast<DeepExperienceMemory>(experience_memory)), 
            policy_net_(policy_net), 
            target_net_(target_net), 
            discount_(discount), 
            horizon_(horizon), 
            batch_size_(batch_size),
            lr_(lr),
            state_space_(std::static_pointer_cast<DiscreteSpace>(state_space)), 
            action_space_(std::static_pointer_cast<MultiDiscreteSpace>(action_space))
    {
        torch::optim::AdamOptions options;
		options.lr(lr);
        this->optimizer_ = std::make_shared<torch::optim::Adam>((*this->policy_net_)->parameters(), options);
    }

    DqnMdpBackup::~DqnMdpBackup()
    {

    }

    double DqnMdpBackup::update()
    {   
        // std::cout << "DqnMdpBackup::update()" << std::endl;

        if (this->experience_memory_->size() < this->batch_size_)
        {
			return 0;
		}

        std::vector<sars_transition> transitions = this->experience_memory_->sample(this->batch_size_);

        auto [t_batch, regular_t_batch, x_batch, u_batch, r_batch, next_t_batch, next_x_batch] = this->constructBatch(transitions);

        // std::cout << std::endl;

        // std::cout << "t_batch" << std::endl;
        // std::cout << t_batch << std::endl;

        // std::cout << "regular_t_batch" << std::endl;
        // std::cout << regular_t_batch << std::endl;

        // std::cout << "x_batch" << std::endl;
        // std::cout << x_batch << std::endl;

        // std::cout << "u_batch" << std::endl;
        // std::cout << u_batch << std::endl;

        // std::cout << "r_batch" << std::endl;
        // std::cout << r_batch << std::endl;

        // std::cout << "next_t_batch" << std::endl;
        // std::cout << next_t_batch << std::endl;

        // std::cout << "next_x_batch" << std::endl;
        // std::cout << next_x_batch << std::endl;

        torch::Tensor q_values = this->getQValues(t_batch, x_batch, u_batch);

        torch::Tensor target_q_values = this->getTargetQValues(next_t_batch, next_x_batch, r_batch, regular_t_batch);


        // std::cout << "q_values" << std::endl;
        // std::cout << q_values << std::endl;

        // std::cout << "target_q_values" << std::endl;
        // std::cout << target_q_values << std::endl;

        torch::Tensor loss = at::smooth_l1_loss(q_values, target_q_values);

        this->optimizer_->zero_grad();

        loss.backward();

        this->optimizer_->step();

        return loss.item<double>();
    }

    std::shared_ptr<Action> DqnMdpBackup::getGreedyAction(const std::shared_ptr<State> &state, number t)
    {
        // std::cout << "DqnMdpBackup::getGreedyAction()" << std::endl;

        // 
        if (t == this->horizon_)
        {
            return nullptr;
        }

        // Let PyTorch know that we don't need to keep track of the gradient in this context.
		torch::NoGradGuard no_grad;
        // Create one hot vector for x with correct number of dimensions.
        number x = this->state_space_->getItemIndex(state);
        torch::Tensor one_hot_x = torch::zeros(this->state_space_->getNumItems());
        one_hot_x[x] = 1;
        // Create one hot vector for t with correct number of dimensions.
        torch::Tensor one_hot_t = torch::zeros(this->horizon_);
        one_hot_t[t] = 1;
        // Concatonate the two.
		torch::Tensor x_t = torch::cat({one_hot_x, one_hot_t});
        // Get u.
        auto u = torch::argmax(this->policy_net_->operator()(x_t)).item<int>();
        // Get the real u from the action space.
        // std::cout << *this->action_space_->getItem(u) << std::endl;
        return this->action_space_->getItem(u)->toAction();
    }

    double DqnMdpBackup::getValueAt(const std::shared_ptr<State> &state, number t)
    {
        // return this->target_net_->getQValuesAt(state, t)->max();
    }

    std::tuple<torch::Tensor, torch::Tensor, torch::Tensor, torch::Tensor, torch::Tensor, torch::Tensor, torch::Tensor> DqnMdpBackup::constructBatch(std::vector<sars_transition> transitions)
    {
        // std::cout << "DqnMdpBackup::constructBatch()" << std::endl;

        std::vector<torch::Tensor> t_batch_vector;
        std::vector<long> regular_t_batch_vector;
        std::vector<torch::Tensor> x_batch_vector;
        std::vector<long> u_batch_vector;
        std::vector<long> r_batch_vector;
        std::vector<torch::Tensor> next_t_batch_vector;
        std::vector<torch::Tensor> next_x_batch_vector;

        for (sars_transition transition: transitions)
        {
            auto [t, x, u, r, next_x, _] = transition;

            torch::Tensor one_hot_t = torch::zeros(this->horizon_);
            one_hot_t[t] = 1;
            t_batch_vector.push_back(one_hot_t);
            regular_t_batch_vector.push_back(t);

            torch::Tensor one_hot_x = torch::zeros(this->state_space_->getNumItems());
            one_hot_x[this->state_space_->getItemIndex(x)] = 1;
            x_batch_vector.push_back(one_hot_x);

            u_batch_vector.push_back(this->action_space_->getItemIndex(u));

            r_batch_vector.push_back(r);

            torch::Tensor one_hot_next_t = torch::zeros(this->horizon_);
            if (t + 1 != this->horizon_)
            {
                one_hot_next_t[t+1] = 1;
            }
            next_t_batch_vector.push_back(one_hot_next_t);

            torch::Tensor one_hot_next_x = torch::zeros(this->state_space_->getNumItems());
            one_hot_next_x[this->state_space_->getItemIndex(next_x)] = 1;
            next_x_batch_vector.push_back(one_hot_next_x);
        }

        torch::Tensor t_batch = torch::cat(t_batch_vector);
        t_batch = t_batch.reshape({this->batch_size_, -1});

        torch::Tensor regular_t_batch = torch::tensor(regular_t_batch_vector);

        torch::Tensor x_batch = torch::cat(x_batch_vector);
        x_batch = x_batch.reshape({this->batch_size_, -1});

        torch::Tensor u_batch = torch::tensor(u_batch_vector);
        u_batch = u_batch.reshape({this->batch_size_, -1});

        torch::Tensor r_batch = torch::tensor(r_batch_vector);

        torch::Tensor next_t_batch = torch::cat(next_t_batch_vector);
        next_t_batch = next_t_batch.reshape({this->batch_size_, -1});
        
        torch::Tensor next_x_batch = torch::cat(next_x_batch_vector);
        next_x_batch = next_x_batch.reshape({this->batch_size_, -1});

        return std::make_tuple(t_batch, regular_t_batch, x_batch, u_batch, r_batch, next_t_batch, next_x_batch);
    }

    torch::Tensor DqnMdpBackup::getQValues(torch::Tensor t_batch, torch::Tensor x_batch, torch::Tensor u_batch)
    {
        // std::cout << "DqnMdpBackup::getQValues()" << std::endl;

        torch::Tensor x_t_batch = torch::cat({x_batch, t_batch}, 1);

        // std::cout << "this->policy_net_->operator()(x_t_batch)" << std::endl;
        // std::cout << this->policy_net_->operator()(x_t_batch) << std::endl;

		return this->policy_net_->operator()(x_t_batch).gather(-1, u_batch);
    }

    torch::Tensor DqnMdpBackup::getTargetQValues(torch::Tensor next_t_batch, torch::Tensor next_x_batch, torch::Tensor r_batch, torch::Tensor regular_t_batch)
    {
        // std::cout << "DqnMdpBackup::getTargetQValues()" << std::endl;

        // Let PyTorch know that we don't need to keep track of the gradient in this context.
		torch::NoGradGuard no_grad;
        torch::Tensor next_x_next_t_batch = torch::cat({next_x_batch, next_t_batch}, 1);
        torch::Tensor next_state_values = std::get<0>(this->target_net_->operator()(next_x_next_t_batch).max(1));
        torch::Tensor zeros = torch::zeros(this->batch_size_);
        torch::Tensor correct_next_state_values = torch::where(regular_t_batch < this->horizon_ - 1, next_state_values, zeros);
        torch::Tensor target_q_values = (correct_next_state_values * this->discount_) + r_batch;
        return target_q_values.unsqueeze(1);
    }
}
#include <sdm/algorithms/deep/osdpomdp/models_update_rules.hpp>


namespace sdm{
	ModelsUpdateRules::ModelsUpdateRules(
		number batch_size, 
		number sampling_memory_size, 
		torch::Device device, 
		std::shared_ptr<sdm::POSG>& game, 
		bool induced_bias
	){
		this->batch_size = batch_size;
		this->sampling_memory_size = sampling_memory_size;
		this->device = device;
		this->game = game;
		this->uniform_alpha_distribution = std::uniform_real_distribution<double>(0.0, 1.0);
		this->induced_bias = induced_bias;
	}

	double ModelsUpdateRules::update(
		std::shared_ptr<ReplayMemory>& replay_memory, 
		std::shared_ptr<Agents>& agents, 
		float alpha
	){
		// If there are not enough transitions in the replay memory to make up even a single batch:
		if (replay_memory->size() < batch_size){
			// Do nothing and return 0 as the loss.
			return 0;
		}
		// Get transitions by randomly sampling the replay memory.
		std::vector<transition> transitions = replay_memory->sample(batch_size);
		// Using these transitions, construct the batch of Tensors.
		batch b = construct_batch(transitions);
		// Initalize the tensors to be able to std::tie them.
		torch::Tensor o2_batch, o1_batch, all_o1s_batch, p_x_batch, u_batch, r_batch, next_o2_batch, next_o1_batch, next_all_o1s_batch, p_next_x_batch;
		std::vector<torch::Tensor> next_all_o1_batches;
		// Std::tie the Tensors appropriately. 
		std::tie(o2_batch, o1_batch, all_o1s_batch, p_x_batch, u_batch, r_batch, next_o2_batch, next_o1_batch, next_all_o1s_batch, next_all_o1_batches, p_next_x_batch) = b; 
		//

		torch::Tensor q_values = get_q_values(o2_batch, o1_batch, all_o1s_batch, p_x_batch, u_batch, agents->policy_net);
		//
		torch::Tensor target_q_values = get_target_q_values(next_o2_batch, next_o1_batch, next_all_o1s_batch, p_next_x_batch, r_batch, agents, alpha);
		//
		torch::Tensor loss = at::smooth_l1_loss(q_values, target_q_values);
		//
		update_policy_net(agents, loss);
		//
		return loss.item<double>();
	}

	torch::Tensor ModelsUpdateRules::get_q_values(torch::Tensor o2_batch, torch::Tensor o1_batch, torch::Tensor all_o1s_batch, torch::Tensor p_x_batch, torch::Tensor u_batch, DQN& policy_net){
		// Construct the input to agent 1's policy net by concatonating o_{t}^{0}, \{o_{t}^{1, (m)}\}, a_{t}^{0}, and \{o_{t}^{1}\}.
		torch::Tensor o2_o1_ao1s_px_batch = torch::cat({o2_batch, o1_batch, all_o1s_batch, p_x_batch}, 1);
		// Put it to agent_1's policy net and get q values for each possible private action. Then using \{a_{t}^{1, (m)}\} select which ones are the relevant ones. Return it.
		return policy_net(o2_o1_ao1s_px_batch).gather(-1, u_batch);
	}

	torch::Tensor ModelsUpdateRules::get_target_q_values(
		torch::Tensor next_o2_batch, 
		torch::Tensor next_o1_batch, 
		torch::Tensor next_all_o1s_batch, 
		torch::Tensor p_next_x_batch, 
		torch::Tensor r_batch, 
		std::shared_ptr<Agents>& agents, 
		float alpha
	){
		// Initalize Tensor for {V}_{t+1}^{1}.
		torch::Tensor next_state_values;
		{
			// Let PyTorch know that we don't need to keep track of the gradient in this context.
			torch::NoGradGuard no_grad;
			// If we don't use the solution of the POMDP as the induced bias.
			if (!induced_bias){
				// Construct the input to agent 1's policy net by concatonating o_{t+1}^{0}, \{o_{t+1}^{1,(m)}\}, a_{t+1}^{0}, \{o_{t+1}^{1}\}.
				torch::Tensor no2_no1_nao1s_pnx_batch = torch::cat({next_o2_batch, next_o1_batch, next_all_o1s_batch, p_next_x_batch}, 1);
				// {V}_{t+1}^{1} = \max_{a'}{PolicyNet}^{Agent1}(o_{t+1}^{0}, \{o_{t+1}^{1, (m)}\}, a_{t+1}^{0}, a', \{o_{t+1}^{1}\})
				next_state_values = std::get<0>(agents->target_net(no2_no1_nao1s_pnx_batch).max(1));
			// If we use the solution of the POMDP as the induced bias.
			} else {
				// We do this with probability alpha.
				if (uniform_alpha_distribution(random_engine) < alpha){
					// Construct the input to agent 1's policy net by concatonating o_{t+1}^{0}, \{o_{t+1}^{1,(m)}\}.
					torch::Tensor no2_no1_batch = torch::cat({next_o2_batch, next_o1_batch}, 1);
					// {V}_{t+1}^{1} = \max_{a'}  {TargetNet}^{InducedBias}(o_{t+1}^{0}, \{o_{t+1}^{1,(m)}\}, a')
					next_state_values = std::get<0>(agents->induced_bias_target_net(no2_no1_batch).max(1));
				// We do this with probability 1-alpha.
				} else {
					// Construct the input to agent 1's policy net by concatonating o_{t+1}^{0}, \{o_{t+1}^{1,(m)}\}, a_{t+1}^{0}, \{o_{t+1}^{1}\}.
					torch::Tensor no2_no1_nao1s_pnx_batch = torch::cat({next_o2_batch, next_o1_batch, next_all_o1s_batch, p_next_x_batch}, 1);
					// {V}_{t+1}^{1} = \max_{a'}{PolicyNet}^{Agent1}(o_{t+1}^{0}, \{o_{t+1}^{1, (m)}\}, a_{t+1}^{0}, a', \{o_{t+1}^{1}\})
					next_state_values = std::get<0>(agents->target_net(no2_no1_nao1s_pnx_batch).max(1));
				}
			}
		}
		// {TargetQ}_{t}^{1} = r_{t} + \gamma \cdot {V}_{t+1}^{1}
		torch::Tensor target_q_values = r_batch + (game->getDiscount() * next_state_values);
		// Add batch dimension and return it.
		return target_q_values.unsqueeze(1);
	}

	void ModelsUpdateRules::update_policy_net(std::shared_ptr<Agents>& agents, torch::Tensor loss){
		// Empy out the gradients.
		agents->optimizer->zero_grad();
		// Backpropagate the gradients of the loss.
		loss.backward();
		// For every parameter in agent 1's policy net:
		for (auto param: agents->policy_net->parameters()){
			// Clamp its gradient between [-1, 1] to avoid ...
			param.grad().data().clamp_(-1, 1);
		}
		// Take one update step.
		agents->optimizer->step();
	}

	// Helper function to contruct the batch of tensors.
	batch ModelsUpdateRules::construct_batch(std::vector<transition> transitions){
		std::vector<torch::Tensor> o2_batch_vector;
		std::vector<torch::Tensor> o1_batch_vector;
		std::vector<torch::Tensor> all_o1s_batch_vector;
		std::vector<torch::Tensor> p_x_batch_vector;
		std::vector<long> u_batch_vector;
		std::vector<long> r_batch_vector;
		std::vector<torch::Tensor> next_o2_batch_vector;
		std::vector<torch::Tensor> next_o1_batch_vector;
		std::vector<torch::Tensor> next_all_o1s_batch_vector;
		std::vector<std::vector<torch::Tensor>> next_all_o1_batches_vector(sampling_memory_size); // sampling_memory x batch_size x dim_o1
		std::vector<torch::Tensor> p_next_x_batch_vector;

		for (transition t: transitions){
			history o2 = std::get<0>(t);
			o2_batch_vector.push_back(o2);

			history o1 = std::get<1>(t);
			o1_batch_vector.push_back(o1);

			std::vector<history> all_o1s = std::get<2>(t);
			torch::Tensor so1s = torch::cat(all_o1s);
			all_o1s_batch_vector.push_back(so1s);

			state_probability_distribution p_x = std::get<3>(t);
			p_x_batch_vector.push_back(p_x);

			action u = std::get<4>(t);
			u_batch_vector.push_back(u);

			reward r = std::get<5>(t);
			r_batch_vector.push_back(r);

			history next_o2 = std::get<6>(t);
			next_o2_batch_vector.push_back(next_o2);

			history next_o1 = std::get<7>(t);
			next_o1_batch_vector.push_back(next_o1);

			std::vector<history> next_all_o1s = std::get<8>(t);
			torch::Tensor nso1s = torch::cat(next_all_o1s);
			next_all_o1s_batch_vector.push_back(nso1s);
			for (int i = 0; i < sampling_memory_size; i++){
				history next_all_o1 = next_all_o1s[i];
				next_all_o1_batches_vector[i].push_back(next_all_o1);
			}

			state_probability_distribution p_next_x = std::get<9>(t);
			p_next_x_batch_vector.push_back(p_next_x);
		}

		torch::Tensor o2_batch = torch::cat(o2_batch_vector);
		o2_batch = o2_batch.reshape({batch_size, -1});
		o2_batch = o2_batch.to(device);

		torch::Tensor o1_batch = torch::cat(o1_batch_vector);
		o1_batch = o1_batch.reshape({batch_size, -1});
		o1_batch = o1_batch.to(device);

		torch::Tensor all_o1s_batch = torch::cat(all_o1s_batch_vector);
		all_o1s_batch = all_o1s_batch.reshape({batch_size, -1});
		all_o1s_batch = all_o1s_batch.to(device);

		torch::Tensor p_x_batch = torch::cat(p_x_batch_vector);
		p_x_batch = p_x_batch.reshape({batch_size, -1});
		p_x_batch = p_x_batch.to(device);

		torch::Tensor u_batch = torch::tensor(u_batch_vector);
		u_batch = u_batch.reshape({batch_size, -1});
		u_batch = u_batch.to(device);

		torch::Tensor r_batch = torch::tensor(r_batch_vector);
		r_batch = r_batch.to(device);

		torch::Tensor next_o2_batch = torch::cat(next_o2_batch_vector);
		next_o2_batch = next_o2_batch.reshape({batch_size, -1});
		next_o2_batch = next_o2_batch.to(device);

		torch::Tensor next_o1_batch = torch::cat(next_o1_batch_vector);
		next_o1_batch = next_o1_batch.reshape({batch_size, -1});
		next_o1_batch = next_o1_batch.to(device);

		torch::Tensor next_all_o1s_batch = torch::cat(next_all_o1s_batch_vector);
		next_all_o1s_batch = next_all_o1s_batch.reshape({batch_size, -1});
		next_all_o1s_batch = next_all_o1s_batch.to(device);
		std::vector<torch::Tensor> next_all_o1_batches;
		for (int i = 0; i < sampling_memory_size; i++){
			std::vector<torch::Tensor> next_one_o1_batch_vector = next_all_o1_batches_vector[i];
			torch::Tensor next_one_o1_batch = torch::cat(next_one_o1_batch_vector);
			next_one_o1_batch = next_one_o1_batch.reshape({batch_size, -1});
			next_one_o1_batch = next_one_o1_batch.to(device);
			next_all_o1_batches.push_back(next_one_o1_batch);
		}

		torch::Tensor p_next_x_batch = torch::cat(p_next_x_batch_vector);
		p_next_x_batch = p_next_x_batch.reshape({batch_size, -1});
		p_next_x_batch = p_next_x_batch.to(device);

		return std::make_tuple(o2_batch, o1_batch, all_o1s_batch, p_x_batch, u_batch, r_batch, next_o2_batch, next_o1_batch, next_all_o1s_batch, next_all_o1_batches, p_next_x_batch);
	}
}
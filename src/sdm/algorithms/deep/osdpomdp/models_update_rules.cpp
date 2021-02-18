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
		torch::Tensor o2_batch, o1_batch, all_o1s_batch, index_u2_batch, u2_batch, index_u1_batch, u1_batch, r_batch, next_o2_batch, next_o1_batch, next_all_o1s_batch;
		std::vector<torch::Tensor> next_all_o1_batches;
		// Std::tie the Tensors appropriately. 
		std::tie(o2_batch, o1_batch, all_o1s_batch, index_u2_batch, u2_batch, index_u1_batch, u1_batch, r_batch, next_o2_batch, next_o1_batch, next_all_o1s_batch, next_all_o1_batches) = b; 
		//
		torch::Tensor q_values = get_q_values(o2_batch, o1_batch, u2_batch, all_o1s_batch, index_u1_batch, agents->agent_1_policy_net);
		//
		torch::Tensor next_u2_batch = get_next_u2_batch(next_o2_batch, next_all_o1s_batch, next_all_o1_batches, agents->agent_1_policy_net);
		//
		torch::Tensor target_q_values = get_target_q_values(next_o2_batch, next_o1_batch, next_u2_batch, next_all_o1s_batch, r_batch, agents, alpha);
		//
		torch::Tensor loss = at::smooth_l1_loss(q_values, target_q_values);
		//
		update_agent_1_policy_net(agents, loss);
		//
		return loss.item<double>();
	}

	torch::Tensor ModelsUpdateRules::get_next_u2_batch(torch::Tensor next_o2_batch, torch::Tensor next_all_o1s_batch, std::vector<torch::Tensor> next_all_o1_batches, DQN& agent_1_policy_net){
		// Initalize Tensor for a_{t+1}^{0}.
		torch::Tensor next_u2_batch;
		{
			// Let PyTorch know that we don't need to keep track of the gradient in this context.
			torch::NoGradGuard no_grad;
			// Initialize {Q}_{t+1}^{0}.
			torch::Tensor q_values = torch::zeros(batch_size * game->getNumActions(0));
			// Reshape it to the correct shape.
			q_values = q_values.reshape({batch_size, -1});
			// Put it to the correct device.
			q_values = q_values.to(device);
			// For each possible a_{t+1}^{0}:
			for(action next_u2 = 0; next_u2 < game->getNumActions(0); next_u2++){
				// The actions must be one-hot-encoded when they are input to networks.
				torch::Tensor one_hot_next_u2 = torch::zeros(game->getNumActions(0) * batch_size);
				// Put it to the correct device.
				one_hot_next_u2 = one_hot_next_u2.to(device);
				// Reshape it to the correct shape.
				one_hot_next_u2 = one_hot_next_u2.reshape({batch_size, -1});
				// This one is for choosing which q values are chosen according to a given a_{t+1}^{0}.
				std::vector<long> index_next_u2_batch_vector;
				// For each value up to batch size:
				for (int i = 0; i < batch_size; i++){
					// Index of at a_{t+1}^{0} is 1, others 0.
					one_hot_next_u2[i][next_u2] = 1;
					// Add a_{t+1}^{0}.
					index_next_u2_batch_vector.push_back(next_u2);
				}
				// Conver it to a Tensor.
				torch::Tensor index_next_u2_batch = torch::tensor(index_next_u2_batch_vector);
				// Reshape it to the correct shape.
				index_next_u2_batch = index_next_u2_batch.reshape({batch_size, -1});
				// Put it to the correct device.
				index_next_u2_batch = index_next_u2_batch.to(device);
				// For each m between 0 to |M|-1:
				for (int m = 0; m < sampling_memory_size; m++){
					// Extract \{o_{t+1}^{1, (m)}\} from \{o_{t+1}^{1}\}.
					torch::Tensor next_one_o1_batch = next_all_o1_batches[m];
					// Construct the input to agent 1's policy net by concatonating o_{t+1}^{0}, \{o_{t+1}^{1, (m)}\}, a_{t+1}^{0}, and \{o_{t+1}^{1}\}.
					torch::Tensor no2_noo1_nu2_nao1s_batch = torch::cat({next_o2_batch, next_one_o1_batch, one_hot_next_u2, next_all_o1s_batch}, 1);
					// 
					q_values.scatter_add_(-1, index_next_u2_batch, std::get<0>(agent_1_policy_net(no2_noo1_nu2_nao1s_batch).max(1)).unsqueeze(1));
				}
			}
			// Divide all the q values by |M|, technically not neccessary but more proper.
			q_values = q_values / sampling_memory_size;
			// Get the maximum q value along each batch.
			torch::Tensor max_q_values = std::get<0>(q_values.max(1)).unsqueeze(1);
			// Contrcut zeros Tensor the same shape as {Q}_{t+1}^{0}.
			torch::Tensor zeros = torch::zeros({batch_size, game->getNumActions(0)});
			// Contrcut ones Tensor the same shape as {Q}_{t+1}^{0}.
			torch::Tensor ones = torch::ones({batch_size, game->getNumActions(0)});
			// Get a_{t+1}^{0} by using torch::where to put 1s where q values where maximal and 0s elsewhere.
			next_u2_batch = torch::where(q_values == max_q_values, ones.to(device), zeros.to(device));
		}
		// Return it.
		return next_u2_batch;
	}

	torch::Tensor ModelsUpdateRules::get_q_values(torch::Tensor o2_batch, torch::Tensor o1_batch, torch::Tensor u2_batch, torch::Tensor all_o1s_batch, torch::Tensor index_u1_batch, DQN& agent_1_policy_net){
		// Construct the input to agent 1's policy net by concatonating o_{t}^{0}, \{o_{t}^{1, (m)}\}, a_{t}^{0}, and \{o_{t}^{1}\}.
		torch::Tensor o2_o1_u2_u21s_batch = torch::cat({o2_batch, o1_batch, u2_batch, all_o1s_batch}, 1);
		// Put it to agent_1's policy net and get q values for each possible private action. Then using \{a_{t}^{1, (m)}\} select which ones are the relevant ones. Return it.
		return agent_1_policy_net(o2_o1_u2_u21s_batch).gather(-1, index_u1_batch);
	}

	torch::Tensor ModelsUpdateRules::get_target_q_values(
		torch::Tensor next_o2_batch, torch::Tensor next_o1_batch, torch::Tensor next_u2_batch, torch::Tensor next_all_o1s_batch, torch::Tensor r_batch, 
		std::shared_ptr<Agents>& agents, float alpha
	){
		// Initalize Tensor for {V}_{t+1}^{1}.
		torch::Tensor next_state_values;
		{
			// Let PyTorch know that we don't need to keep track of the gradient in this context.
			torch::NoGradGuard no_grad;
			// If we don't use the solution of the POMDP as the induced bias.
			if (!induced_bias){
				// Construct the input to agent 1's policy net by concatonating o_{t+1}^{0}, \{o_{t+1}^{1,(m)}\}, a_{t+1}^{0}, \{o_{t+1}^{1}\}.
				torch::Tensor no2_no1_nu2_nao1s_batch = torch::cat({next_o2_batch, next_o1_batch, next_u2_batch, next_all_o1s_batch}, 1);
				// {V}_{t+1}^{1} = \max_{a'}{PolicyNet}^{Agent1}(o_{t+1}^{0}, \{o_{t+1}^{1, (m)}\}, a_{t+1}^{0}, a', \{o_{t+1}^{1}\})
				next_state_values = std::get<0>(agents->agent_1_target_net(no2_no1_nu2_nao1s_batch).max(1));
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
					torch::Tensor no2_no1_nu2_nao1s_batch = torch::cat({next_o2_batch, next_o1_batch, next_u2_batch, next_all_o1s_batch}, 1);
					// {V}_{t+1}^{1} = \max_{a'}{PolicyNet}^{Agent1}(o_{t+1}^{0}, \{o_{t+1}^{1, (m)}\}, a_{t+1}^{0}, a', \{o_{t+1}^{1}\})
					next_state_values = std::get<0>(agents->agent_1_target_net(no2_no1_nu2_nao1s_batch).max(1));
				}
			}
		}
		// {TargetQ}_{t}^{1} = r_{t} + \gamma \cdot {V}_{t+1}^{1}
		torch::Tensor target_q_values = r_batch + (game->getDiscount() * next_state_values);
		// Add batch dimension and return it.
		return target_q_values.unsqueeze(1);
	}

	void ModelsUpdateRules::update_agent_1_policy_net(std::shared_ptr<Agents>& agents, torch::Tensor loss){
		// Empy out the gradients.
		agents->optimizer->zero_grad();
		// Backpropagate the gradients of the loss.
		loss.backward();
		// For every parameter in agent 1's policy net:
		for (auto param: agents->agent_1_policy_net->parameters()){
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
		std::vector<long> index_u2_batch_vector;
		std::vector<torch::Tensor> u2_batch_vector;
		std::vector<long> index_u1_batch_vector;
		std::vector<torch::Tensor> u1_batch_vector;
		std::vector<long> r_batch_vector;
		std::vector<torch::Tensor> next_o2_batch_vector;
		std::vector<torch::Tensor> next_o1_batch_vector;
		std::vector<torch::Tensor> next_all_o1s_batch_vector;
		std::vector<std::vector<torch::Tensor>> next_all_o1_batches_vector(sampling_memory_size); // sampling_memory x batch_size x dim_o1

		for (transition t: transitions){
			history o2 = std::get<0>(t);
			o2_batch_vector.push_back(o2);

			history o1 = std::get<1>(t);
			o1_batch_vector.push_back(o1);

			std::vector<history> all_o1s = std::get<2>(t);
			torch::Tensor so1s = torch::cat(all_o1s);
			all_o1s_batch_vector.push_back(so1s);

			action u2 = std::get<3>(t);
			index_u2_batch_vector.push_back(u2);
			torch::Tensor one_hot_u2 = torch::zeros(game->getNumActions(0));
			one_hot_u2[u2] = 1;
			u2_batch_vector.push_back(one_hot_u2);

			action u1 = std::get<4>(t);
			index_u1_batch_vector.push_back(u1);
			torch::Tensor one_hot_u1 = torch::zeros(game->getNumActions(1));
			one_hot_u1[u1] = 1;
			u1_batch_vector.push_back(one_hot_u1);

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

		torch::Tensor index_u2_batch = torch::tensor(index_u2_batch_vector);
		index_u2_batch = index_u2_batch.reshape({batch_size, -1});
		index_u2_batch = index_u2_batch.to(device);
		torch::Tensor u2_batch = torch::cat(u2_batch_vector);
		u2_batch = u2_batch.reshape({batch_size, -1});
		u2_batch = u2_batch.to(device);

		torch::Tensor index_u1_batch = torch::tensor(index_u1_batch_vector);
		index_u1_batch = index_u1_batch.reshape({batch_size, -1});
		index_u1_batch = index_u1_batch.to(device);
		torch::Tensor u1_batch = torch::cat(u1_batch_vector);
		u1_batch = u1_batch.reshape({batch_size, -1});
		u1_batch = u1_batch.to(device);

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

		return std::make_tuple(o2_batch, o1_batch, all_o1s_batch, index_u2_batch, u2_batch, index_u1_batch, u1_batch, r_batch, next_o2_batch, next_o1_batch, next_all_o1s_batch, next_all_o1_batches);
	}
}
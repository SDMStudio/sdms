#include <sdm/algorithms/deep/osdpomdp/models_update_rules.hpp>


namespace sdm{
	ModelsUpdateRules::ModelsUpdateRules(
		number batch_size, 
		number K, 
		number seed,
		torch::Device device, 
		std::shared_ptr<sdm::POSG>& game, 
		bool induced_bias
	){
		this->batch_size = batch_size;
		this->K = K;
		this->device = device;
		this->game = game;
		this->uniform_alpha_distribution = std::uniform_real_distribution<double>(0.0, 1.0); // not used atm
		this->random_engine.seed(seed); // not used atm 
		this->induced_bias = induced_bias; // not used atm
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
		// Using these transitions, construct the batch.
		batch b = construct_batch(transitions);
		// Initalize the batches.
		torch::Tensor o2_batch, o1_batch, o1s_batch, u_batch, r_batch, next_o2_batch, next_o1_batch, next_o1s_batch;
		// Same data as next_o1s_batch but in a different format for ease of access.
		std::vector<torch::Tensor> next_o1_batches_vector;
		// Match the data from the batch appropriately.
		std::tie(o2_batch, o1_batch, o1s_batch, u_batch, r_batch, next_o2_batch, next_o1_batch, next_o1s_batch, next_o1_batches_vector) = b; 
		//
		torch::Tensor q_values = get_q_values(o2_batch, o1_batch, o1s_batch, u_batch, agents->policy_net);
		//
		torch::Tensor target_q_values = get_target_q_values(next_o2_batch, next_o1_batch, next_o1_batches_vector, next_o1s_batch, r_batch, agents->target_net);
		//
		torch::Tensor loss = at::smooth_l1_loss(q_values, target_q_values);
		//
		update_policy_net(agents, loss);
		// Return the loss.
		return loss.item<double>();
	}

	torch::Tensor ModelsUpdateRules::get_next_u2_batch(
		torch::Tensor next_o2_batch,
		std::vector<torch::Tensor> next_o1_batches_vector, 
		torch::Tensor next_o1s_batch,
		DQN& target_net
	){
		// Let PyTorch know that we don't need to keep track of the gradient in this context.
		torch::NoGradGuard no_grad;
		// Create Next Q Values tensor for agent 2, reshape it to add batch dimension, put it to correct device.
		torch::Tensor next_q_values_agent_2 = torch::zeros(batch_size * game->getNumActions(0)).reshape({batch_size, -1}).to(device);
		// For each possible next u2:
		for(action next_u2 = 0; next_u2 < game->getNumActions(0); next_u2++){
			// Vector for gathering data to conctruct next u2 batch.
			std::vector<int> next_u2_batch_vector;
			// For each entry in the batch:
			for (int i = 0; i < batch_size; i++){
					// Add the same next u2.
					next_u2_batch_vector.push_back(next_u2);
			}
			// Convert it to a Tensor, reshape it to add batch dimension, put it to correct device.
			torch::Tensor next_u2_batch = torch::tensor(next_u2_batch_vector).reshape({batch_size, -1}).to(device);
			// For each k:
			for (int k = 0; k < K; k++){
				// Get next u1 given next o2, next o1, next o1s, next u2.
				torch::Tensor next_u1_batch = get_next_u1_batch(next_o2_batch, next_o1_batches_vector[k], next_o1s_batch, next_u2_batch, target_net);
				// Get next joint action batch, given batches of private actions of agent 2 and 1.
				torch::Tensor next_u_batch = get_next_u_batch(next_u2_batch, next_u1_batch);
				// Create tensor to be put to the model by concatenating next o2, next o1, and next o1s along the batch dimension (1).
				torch::Tensor no2_no1_no1s_batch = torch::cat({next_o2_batch, next_o1_batches_vector[k], next_o1s_batch}, 1);
				// Get the Next Q values for joint actions.
				torch::Tensor next_q_values = target_net(no2_no1_no1s_batch);
				// Select the Next Q values using joint actions as index, 
				// then add them to the correct place using agent 2's private actions as index.
				next_q_values_agent_2.scatter_add_(-1, next_u2_batch, next_q_values.gather(-1, next_u_batch));
			}
		}
		// Get the argmax of Next Q values along batch dimension (1), add batch dimension, return it.
		return next_q_values_agent_2.argmax(1).reshape({batch_size, -1});
	}

	torch::Tensor ModelsUpdateRules::get_next_u1_batch(
		torch::Tensor next_o2_batch, 
		torch::Tensor next_o1_batch, 
		torch::Tensor next_o1s_batch, 
		torch::Tensor next_u2_batch, 
		DQN& target_net
	){
		// Let PyTorch know that we don't need to keep track of the gradient in this context.
		torch::NoGradGuard no_grad;
		// Create Next Q Values tensor for agent 1, reshape it to add batch dimension, put it to correct device.
		torch::Tensor next_q_values_agent_1 = torch::zeros(batch_size * game->getNumActions(1)).reshape({batch_size, -1}).to(device);
		// Create tensor to be put to the model by concatenating next o2, next o1, and next o1s along the batch dimension (1).
		torch::Tensor no2_no1_no1s_batch = torch::cat({next_o2_batch, next_o1_batch, next_o1s_batch}, 1);
		// Get the Next Q values for joint actions.
		torch::Tensor next_q_values = target_net(no2_no1_no1s_batch);
		// For each possible next u1:
		for(action next_u1 = 0; next_u1 < game->getNumActions(1); next_u1++){
			// Vector for gathering data to conctruct next u1 batch.
			std::vector<int> next_u1_batch_vector;
			// For each entry in the batch:
			for (int i = 0; i < batch_size; i++){
					// Add the same next u1.
					next_u1_batch_vector.push_back(next_u1);
			}
			// Convert it to a Tensor, reshape it to add batch dimension, put it to correct device.
			torch::Tensor next_u1_batch = torch::tensor(next_u1_batch_vector).reshape({batch_size, -1}).to(device);
			// Get next joint action batch, given batches of private actions of agent 2 and 1.
			torch::Tensor next_u_batch = get_next_u_batch(next_u2_batch, next_u1_batch);
			// Select the Next Q values using joint actions as index, 
			// then put them to the correct place using agent 1's private actions as index.
			next_q_values_agent_1.scatter_add_(-1, next_u1_batch, next_q_values.gather(-1, next_u_batch));
		}
		// Get the argmax of Next Q values of agent 1 along batch dimension (1), add batch dimension, return it.
		return next_q_values_agent_1.argmax(1).reshape({batch_size, -1});
	}

	torch::Tensor ModelsUpdateRules::get_next_u_batch(torch::Tensor next_u2_batch, torch::Tensor next_u1_batch){
		// Vector for gathering data to conctruct next u batch.
		std::vector<int> next_u_batch_vector;
		// Put these to CPU since otherwise it'll be way too slow to do the next thing.
		next_u2_batch = next_u2_batch.cpu();
		next_u1_batch = next_u1_batch.cpu();
			// For each entry in the batch:
			for (int i = 0; i < batch_size; i++){
				// Get the next_u2 at position i.
				action next_u2 = next_u2_batch.index({i}).item<int>();
				// Get the next_u1 at position i.
				action next_u1 = next_u1_batch.index({i}).item<int>();
				// Calculate next_u given next_u2 and next_u1.
				action next_u = game->getActionSpace().joint2single({next_u2, next_u1});
				// Append next_u.
				next_u_batch_vector.push_back(next_u);
			}
			// Convert it to a Tensor, reshape it to add batch dimension, put it to correct device, return it.
			return torch::tensor(next_u_batch_vector).reshape({batch_size, -1}).to(device);
	}

	torch::Tensor ModelsUpdateRules::get_q_values(torch::Tensor o2_batch, torch::Tensor o1_batch, torch::Tensor o1s_batch, torch::Tensor u_batch, DQN& policy_net){
		// Construct the input for the net by concatonating o2, o1, and o1s batch along batch dimension (1).
		torch::Tensor o2_o1_o1s_batch = torch::cat({o2_batch, o1_batch, o1s_batch}, 1);
		// Put it to policy net and get q values for each possible joint action. Then using ... select which ones are the relevant ones. Return it.
		return policy_net(o2_o1_o1s_batch).gather(-1, u_batch);
	}

	torch::Tensor ModelsUpdateRules::get_target_q_values(
		torch::Tensor next_o2_batch, 
		torch::Tensor next_o1_batch, 
		std::vector<torch::Tensor> next_o1_batches_vector, 
		torch::Tensor next_o1s_batch, 
		torch::Tensor r_batch, 
		DQN& target_net
	){
		// Let PyTorch know that we don't need to keep track of the gradient in this context.
		torch::NoGradGuard no_grad;
		// Get next u2 given next o2 and next o1s.
		torch::Tensor next_u2_batch = get_next_u2_batch(next_o2_batch, next_o1_batches_vector, next_o1s_batch, target_net);
		// Get next u1 given next o2, next o1, next o1s, next u2.
		torch::Tensor next_u1_batch = get_next_u1_batch(next_o2_batch, next_o1_batch, next_o1s_batch, next_u2_batch, target_net);
		// Get next joint action batch, given batches of private actions of agent 2 and 1.
		torch::Tensor next_u_batch = get_next_u_batch(next_u2_batch, next_u1_batch);
		// Create tensor to be put to the model by concatenating next o2, next o1, and next o1s along the batch dimension (1).
		torch::Tensor no2_no1_no1s_batch = torch::cat({next_o2_batch, next_o1_batch, next_o1s_batch}, 1);
		// torch::Tensor next_state_values = std::get<0>(target_net(no2_no1_no1s_batch).max(1));
		torch::Tensor next_q_values = target_net(no2_no1_no1s_batch).gather(-1, next_u_batch).squeeze();
		// {TargetQ}_{t} = r_{t} + \gamma \cdot {Q}_{t+1}
		torch::Tensor target_q_values = r_batch + (game->getDiscount() * next_q_values);
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
		// Vector for gathering data to conctruct o2 batch.
		std::vector<torch::Tensor> o2_batch_vector;
		// Vector for gathering data to conctruct o1 batch.
		std::vector<torch::Tensor> o1_batch_vector;
		// Vector for gathering data to conctruct o1s batch.
		std::vector<torch::Tensor> o1s_batch_vector;
		// Vector for gathering data to conctruct u batch.
		std::vector<int> u_batch_vector;
		// Vector for gathering data to conctruct r batch.
		std::vector<long> r_batch_vector;
		// Vector for gathering data to conctruct next o2 batch.
		std::vector<torch::Tensor> next_o2_batch_vector;
		// Vector for gathering data to conctruct next o1 batch.
		std::vector<torch::Tensor> next_o1_batch_vector;
		// Vector for gathering data to conctruct next o1s batch.
		std::vector<torch::Tensor> next_o1s_batch_vector;
		// Vector for gathering data to conctruct next o1 batches vector. Same data ad above but in a different format.
		std::vector<std::vector<torch::Tensor>> next_o1_batches_vector_vector(K); // K x batch_size x dim_o1
		// For each transition:
		for (transition t: transitions){
			history o2 = std::get<0>(t);
			o2_batch_vector.push_back(o2);

			history o1 = std::get<1>(t);
			o1_batch_vector.push_back(o1);

			std::vector<history> o1s = std::get<2>(t);
			o1s_batch_vector.push_back(torch::cat(o1s));

			action u = std::get<3>(t);
			u_batch_vector.push_back(u);

			reward r = std::get<4>(t);
			r_batch_vector.push_back(r);

			history next_o2 = std::get<5>(t);
			next_o2_batch_vector.push_back(next_o2);

			history next_o1 = std::get<6>(t);
			next_o1_batch_vector.push_back(next_o1);

			std::vector<history> next_o1s = std::get<7>(t);
			next_o1s_batch_vector.push_back(torch::cat(next_o1s));
			for (int k = 0; k < K; k++){
				next_o1_batches_vector_vector[k].push_back(next_o1s[k]);
			}
		}

		torch::Tensor o2_batch = torch::cat(o2_batch_vector).reshape({batch_size, -1}).to(device);

		torch::Tensor o1_batch = torch::cat(o1_batch_vector).reshape({batch_size, -1}).to(device);

		torch::Tensor o1s_batch = torch::cat(o1s_batch_vector).reshape({batch_size, -1}).to(device);

		torch::Tensor u_batch = torch::tensor(u_batch_vector).reshape({batch_size, -1}).to(device);

		torch::Tensor r_batch = torch::tensor(r_batch_vector).to(device);

		torch::Tensor next_o2_batch = torch::cat(next_o2_batch_vector).reshape({batch_size, -1}).to(device);

		torch::Tensor next_o1_batch = torch::cat(next_o1_batch_vector).reshape({batch_size, -1}).to(device);

		torch::Tensor next_o1s_batch = torch::cat(next_o1s_batch_vector).reshape({batch_size, -1}).to(device);
		std::vector<torch::Tensor> next_o1_batches_vector;
		for (int k = 0; k < K; k++){
			next_o1_batches_vector.push_back(torch::cat(next_o1_batches_vector_vector[k]).reshape({batch_size, -1}).to(device));
		}

		return std::make_tuple(o2_batch, o1_batch, o1s_batch, u_batch, r_batch, next_o2_batch, next_o1_batch, next_o1s_batch, next_o1_batches_vector);
	}
}
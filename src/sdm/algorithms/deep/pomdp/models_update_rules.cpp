#include <sdm/algorithms/deep/pomdp/models_update_rules.hpp>


namespace sdm{
	POMDP_ModelsUpdateRules::POMDP_ModelsUpdateRules(
		number batch_size, 
		number tao, 
		number eta, 
		torch::Device device, 
		std::shared_ptr<sdm::POSG>& game,
		bool zerod
	){
		this->batch_size = batch_size;
		this->tao = tao;
		this->eta = eta;
		this->device = device;
		this->game = game;
		this->zerod = zerod;
		this->uniform_tao_distribution = std::uniform_int_distribution<int>(1, tao);
	}

	double POMDP_ModelsUpdateRules::update(
		std::shared_ptr<POMDP_ReplayMemory>& replay_memory, 
		std::shared_ptr<POMDP_Agents>& agents
	){

		if (replay_memory->size() * eta < batch_size){
			return 0;
		}

		std::vector<pomdp_transitions_sequence> transition_sequences = replay_memory->sample();

		// C++ being annoying, i have to declare these here
		torch::Tensor next_o2_batch, next_o1_batch;

		// Q Value Loss
		torch::Tensor loss = torch::zeros({1});
		loss = loss.to(device);

		// int tao_star = uniform_tao_distribution(random_engine);

		for (int t = 0; t < tao; t++){

			pomdp_batch b = construct_batch(transition_sequences[t]);

			torch::Tensor o2_batch, o1_batch, u2_batch, u1_batch, index_u2_u1_batch, z2_batch, z1_batch, r_batch;
			
			std::tie(o2_batch, o1_batch, u2_batch, u1_batch, index_u2_u1_batch, z2_batch, z1_batch, r_batch) = b;

			if (t != 0){
				o2_batch = next_o2_batch;
				o1_batch = next_o1_batch;
			}			

			next_o2_batch = get_next_history_batch(u2_batch, z2_batch, o2_batch, agents->policy_nets->trans_net_2);

			next_o1_batch = get_next_history_batch(u1_batch, z1_batch, o1_batch, agents->policy_nets->trans_net_1);

			torch::Tensor q_values = get_q_values(o2_batch, o1_batch, index_u2_u1_batch, agents->policy_nets->q_net);

			torch::Tensor target_next_o2_batch, target_next_o1_batch;
			{
				torch::NoGradGuard no_grad;
				target_next_o2_batch = get_next_history_batch(u2_batch, z2_batch, o2_batch, agents->target_nets->trans_net_2);
				target_next_o1_batch = get_next_history_batch(u1_batch, z1_batch, o1_batch, agents->target_nets->trans_net_1);
			}
			
			torch::Tensor target_q_values = get_target_q_values(target_next_o2_batch, target_next_o1_batch, r_batch, agents->target_nets->q_net);
			
			loss += at::smooth_l1_loss(q_values, target_q_values);
			
		}

		update_nets(agents, loss);
		// Return the loss divided by tao to make it fair
		return loss.item<double>() / tao;
	}

	torch::Tensor POMDP_ModelsUpdateRules::get_next_history_batch(torch::Tensor u_batch, torch::Tensor z_batch, torch::Tensor o_batch, RNN& transition_net){
		torch::Tensor u_z_batch = torch::cat({u_batch, z_batch}, 1);
		return transition_net(u_z_batch, o_batch);
	}

	torch::Tensor POMDP_ModelsUpdateRules::get_q_values(torch::Tensor o2_batch, torch::Tensor o1_batch, torch::Tensor index_u2_u1_batch, DQN& policy_net){
		torch::Tensor o2_o1_batch = torch::cat({o2_batch, o1_batch}, 1);
		return policy_net(o2_o1_batch).gather(-1, index_u2_u1_batch);
	}

	torch::Tensor POMDP_ModelsUpdateRules::get_target_q_values(torch::Tensor next_o2_batch, torch::Tensor next_o1_batch, torch::Tensor r_batch, DQN& target_net){
		torch::Tensor no2_no1_batch = torch::cat({next_o2_batch, next_o1_batch}, 1);
		torch::NoGradGuard no_grad;
		torch::Tensor next_state_values = std::get<0>(target_net(no2_no1_batch).max(1));
		torch::Tensor target_q_values = (next_state_values * game->getDiscount()) + r_batch;
		// Add batch dimension and return it.
		return target_q_values.unsqueeze(1);
	}

	// void POMDP_ModelsUpdateRules::update_nets(std::shared_ptr<POMDP_Agents>& agents, torch::Tensor loss){
	void POMDP_ModelsUpdateRules::update_nets(std::shared_ptr<POMDP_Agents>& agents, torch::Tensor loss){
		// Empy out the gradients.
		agents->optimizer->zero_grad();
		// Backpropagate the gradients of the loss.
		loss.backward();
		// For every parameter in 
		// for (auto param: agents->policy_nets->q_net->parameters()){
		for (auto param: agents->policy_nets->q_net->parameters()){
			// Clamp its gradient between [-1, 1] to avoid ...
			param.grad().data().clamp_(-1, 1);
		}
		// Take one update step.
		agents->optimizer->step();
	}

	// Helper function to contruct the batch of tensors.
	pomdp_batch POMDP_ModelsUpdateRules::construct_batch(std::vector<pomdp_transition> transitions){
		std::vector<torch::Tensor> o2_batch_vector;
		std::vector<torch::Tensor> o1_batch_vector;
		std::vector<torch::Tensor> u2_batch_vector;
		std::vector<torch::Tensor> u1_batch_vector;
		std::vector<long> index_u2_u1_batch_vector;
		std::vector<torch::Tensor> z2_batch_vector;
		std::vector<torch::Tensor> z1_batch_vector;
		std::vector<long> r_batch_vector;

		for (pomdp_transition t: transitions){
			history o2 = std::get<0>(t);
			history o1 = std::get<1>(t);
			action u2 = std::get<2>(t);
			action u1 = std::get<3>(t);
			action u2_u1 = std::get<4>(t);
			observation z2 = std::get<5>(t);
			observation z1 = std::get<6>(t);
			reward r = std::get<7>(t);

			o2_batch_vector.push_back(o2);

			o1_batch_vector.push_back(o1);

			torch::Tensor one_hot_u2 = torch::zeros(game->getNumActions(0));
			one_hot_u2[u2] = 1;
			u2_batch_vector.push_back(one_hot_u2);

			torch::Tensor one_hot_u1 = torch::zeros(game->getNumActions(0));
			one_hot_u1[u1] = 1;
			u1_batch_vector.push_back(one_hot_u1);

			index_u2_u1_batch_vector.push_back(u2_u1);

			torch::Tensor one_hot_z2 = torch::zeros(game->getNumObservations(0));
			one_hot_z2[z2] = 1;
			z2_batch_vector.push_back(one_hot_z2);

			torch::Tensor one_hot_z1 = torch::zeros(game->getNumObservations(1));
			one_hot_z1[z1] = 1;
			z1_batch_vector.push_back(one_hot_z1);

			r_batch_vector.push_back(r);
		}
		
		torch::Tensor o2_batch = torch::cat(o2_batch_vector);
		if (zerod){
			o2_batch = torch::zeros_like(o2_batch);
		}
		o2_batch = o2_batch.reshape({batch_size, -1});
		o2_batch = o2_batch.to(device);

		torch::Tensor o1_batch = torch::cat(o1_batch_vector);
		if (zerod){
			o1_batch = torch::zeros_like(o1_batch);
		}
		o1_batch = o1_batch.reshape({batch_size, -1});
		o1_batch = o1_batch.to(device);

		torch::Tensor u2_batch = torch::cat(u2_batch_vector);
		u2_batch = u2_batch.reshape({batch_size, -1});
		u2_batch = u2_batch.to(device);

		torch::Tensor u1_batch = torch::cat(u1_batch_vector);
		u1_batch = u1_batch.reshape({batch_size, -1});
		u1_batch = u1_batch.to(device);

		torch::Tensor index_u2_u1_batch = torch::tensor(index_u2_u1_batch_vector);
		index_u2_u1_batch = index_u2_u1_batch.reshape({batch_size, -1});
		index_u2_u1_batch = index_u2_u1_batch.to(device);

		torch::Tensor z2_batch = torch::cat(z2_batch_vector);
		z2_batch = z2_batch.reshape({batch_size, -1});
		z2_batch = z2_batch.to(device);

		torch::Tensor z1_batch = torch::cat(z1_batch_vector);
		z1_batch = z1_batch.reshape({batch_size, -1});
		z1_batch = z1_batch.to(device);

		torch::Tensor r_batch = torch::tensor(r_batch_vector);
		r_batch = r_batch.to(device);

		return std::make_tuple(o2_batch, o1_batch, u2_batch, u1_batch, index_u2_u1_batch, z2_batch, z1_batch, r_batch);
	}
}
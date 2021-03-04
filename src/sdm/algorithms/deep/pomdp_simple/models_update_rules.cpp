#include <sdm/algorithms/deep/pomdp_simple/models_update_rules.hpp>


namespace sdm{
	POMDP_ModelsUpdateRules_Simple::POMDP_ModelsUpdateRules_Simple(
		number batch_size, 
		torch::Device device, 
		std::shared_ptr<sdm::POSG>& game
	){
		this->batch_size = batch_size;
		this->device = device;
		this->game = game;
	}

	double POMDP_ModelsUpdateRules_Simple::update(
		std::shared_ptr<POMDP_ReplayMemory_Simple>& replay_memory, 
		std::shared_ptr<POMDP_Agents_Simple>& agents
	){
		if (replay_memory->size() < batch_size){
			return 0;
		}

		std::vector<pomdp_transition_simple> transitions = replay_memory->sample();

		pomdp_batch_simple b = construct_batch(transitions);

		torch::Tensor o2_batch, o1_batch, index_u2_u1_batch, r_batch, next_o2_batch, next_o1_batch;
		
		std::tie(o2_batch, o1_batch, index_u2_u1_batch, r_batch, next_o2_batch, next_o1_batch) = b;		

		torch::Tensor q_values = get_q_values(o2_batch, o1_batch, index_u2_u1_batch, agents->policy_net);
		
		torch::Tensor target_q_values = get_target_q_values(next_o2_batch, next_o1_batch, r_batch, agents->target_net);
		
		torch::Tensor loss = at::smooth_l1_loss(q_values, target_q_values);			

		update_net(agents, loss);
		// Return the loss
		return loss.item<double>();
	}

	torch::Tensor POMDP_ModelsUpdateRules_Simple::get_q_values(torch::Tensor o2_batch, torch::Tensor o1_batch, torch::Tensor index_u2_u1_batch, DQN& policy_net){
		torch::Tensor o2_o1_batch = torch::cat({o2_batch, o1_batch}, 1);
		return policy_net(o2_o1_batch).gather(-1, index_u2_u1_batch);
	}

	torch::Tensor POMDP_ModelsUpdateRules_Simple::get_target_q_values(torch::Tensor next_o2_batch, torch::Tensor next_o1_batch, torch::Tensor r_batch, DQN& target_net){
		torch::Tensor no2_no1_batch = torch::cat({next_o2_batch, next_o1_batch}, 1);
		torch::NoGradGuard no_grad;
		torch::Tensor next_state_values = std::get<0>(target_net(no2_no1_batch).max(1));
		torch::Tensor target_q_values = (next_state_values * game->getDiscount()) + r_batch;
		// Add batch dimension and return it.
		return target_q_values.unsqueeze(1);
	}

	// void POMDP_ModelsUpdateRules_Simple::update_net(std::shared_ptr<POMDP_Agents>& agents, torch::Tensor loss){
	void POMDP_ModelsUpdateRules_Simple::update_net(std::shared_ptr<POMDP_Agents_Simple>& agents, torch::Tensor loss){
		// Empy out the gradients.
		agents->optimizer->zero_grad();
		// Backpropagate the gradients of the loss.
		loss.backward();
		// For every parameter in 
		// for (auto param: agents->policy_nets->q_net->parameters()){
		for (auto param: agents->policy_net->parameters()){
			// Clamp its gradient between [-1, 1] to avoid ...
			param.grad().data().clamp_(-1, 1);
		}
		// Take one update step.
		agents->optimizer->step();
	}

	// Helper function to contruct the batch of tensors.
	pomdp_batch_simple POMDP_ModelsUpdateRules_Simple::construct_batch(std::vector<pomdp_transition_simple> transitions){
		std::vector<torch::Tensor> o2_batch_vector;
		std::vector<torch::Tensor> o1_batch_vector;
		std::vector<torch::Tensor> u2_batch_vector;
		std::vector<torch::Tensor> u1_batch_vector;
		std::vector<long> index_u2_u1_batch_vector;
		std::vector<torch::Tensor> z2_batch_vector;
		std::vector<torch::Tensor> z1_batch_vector;
		std::vector<long> r_batch_vector;
		std::vector<torch::Tensor> next_o2_batch_vector;
		std::vector<torch::Tensor> next_o1_batch_vector;

		for (pomdp_transition_simple t: transitions){
			history o2 = std::get<0>(t);
			history o1 = std::get<1>(t);
			action u2_u1 = std::get<2>(t);
			reward r = std::get<3>(t);
			history next_o2 = std::get<4>(t);
			history next_o1 = std::get<5>(t);

			o2_batch_vector.push_back(o2);

			o1_batch_vector.push_back(o1);

			index_u2_u1_batch_vector.push_back(u2_u1);

			r_batch_vector.push_back(r);

			next_o2_batch_vector.push_back(o2);

			next_o1_batch_vector.push_back(o1);
		}
		
		torch::Tensor o2_batch = torch::cat(o2_batch_vector);
		o2_batch = o2_batch.reshape({batch_size, -1});
		o2_batch = o2_batch.to(device);

		torch::Tensor o1_batch = torch::cat(o1_batch_vector);
		o1_batch = o1_batch.reshape({batch_size, -1});
		o1_batch = o1_batch.to(device);

		torch::Tensor index_u2_u1_batch = torch::tensor(index_u2_u1_batch_vector);
		index_u2_u1_batch = index_u2_u1_batch.reshape({batch_size, -1});
		index_u2_u1_batch = index_u2_u1_batch.to(device);

		torch::Tensor r_batch = torch::tensor(r_batch_vector);
		r_batch = r_batch.to(device);

		torch::Tensor next_o2_batch = torch::cat(next_o2_batch_vector);
		next_o2_batch = next_o2_batch.reshape({batch_size, -1});
		next_o2_batch = next_o2_batch.to(device);

		torch::Tensor next_o1_batch = torch::cat(next_o1_batch_vector);
		next_o1_batch = next_o1_batch.reshape({batch_size, -1});
		next_o1_batch = next_o1_batch.to(device);

		return std::make_tuple(o2_batch, o1_batch, index_u2_u1_batch, r_batch, next_o2_batch, next_o1_batch);
	}
}
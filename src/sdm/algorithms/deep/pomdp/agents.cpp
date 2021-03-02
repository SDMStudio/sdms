#include <sdm/algorithms/deep/pomdp/agents.hpp>

namespace sdm{
	POMDP_Agents::POMDP_Agents(
		number trans_net_2_input_dim, number trans_net_2_hidden_dim, 
		number trans_net_1_input_dim, number trans_net_1_hidden_dim, 
		number q_net_input_dim, number q_net_inner_dim, number q_net_output_dim, 
		std::shared_ptr<sdm::POSG>& game, torch::Device device, float lr, float adam_eps, std::string ib_net_filename
	){
		this->policy_nets = Deep_Recurrent_Q_Network(
			trans_net_2_input_dim, trans_net_2_hidden_dim, 
			trans_net_1_input_dim, trans_net_1_hidden_dim, 
			q_net_input_dim, q_net_inner_dim, q_net_output_dim
		);
		this->policy_nets->to(device);

		this->target_nets = Deep_Recurrent_Q_Network(
			trans_net_2_input_dim, trans_net_2_hidden_dim, 
			trans_net_1_input_dim, trans_net_1_hidden_dim, 
			q_net_input_dim, q_net_inner_dim, q_net_output_dim
		);
		this->target_nets->to(device);

		this->uniform_epsilon_distribution = std::uniform_real_distribution<double>(0.0, 1.0);
		this->uniform_action_distribution = std::uniform_int_distribution<int>(0, game->getNumActions(0) * game->getNumActions(1) - 1);
		torch::optim::AdamOptions options;
		options.eps(adam_eps);
		options.lr(lr);
		this->optimizer = std::make_shared<torch::optim::Adam>(policy_nets->parameters(), options);
		this->device = device;
		this->game = game;
		this->ib_net_filename = ib_net_filename;
		update_target_nets();
	}

	void POMDP_Agents::save_induced_bias(){
		std::string ib_transition_net_2_filename;
		ib_transition_net_2_filename.append("../models");
		ib_transition_net_2_filename.append("/");
		ib_transition_net_2_filename.append(ib_net_filename);
		ib_transition_net_2_filename.append("_transition_net_2.pt");
		torch::save(policy_nets->trans_net_2, ib_transition_net_2_filename);


		std::string ib_transition_net_1_filename;
		ib_transition_net_1_filename.append("../models");
		ib_transition_net_1_filename.append("/");
		ib_transition_net_1_filename.append(ib_net_filename);
		ib_transition_net_1_filename.append("_transition_net_1.pt");
		torch::save(policy_nets->trans_net_1, ib_transition_net_1_filename);

		std::string ib_target_net_filename;
		ib_target_net_filename.append("../models");
		ib_target_net_filename.append("/");
		ib_target_net_filename.append(ib_net_filename);
		ib_target_net_filename.append("_target_net.pt");
		torch::save(policy_nets->q_net, ib_target_net_filename);
	}

	action POMDP_Agents::get_epsilon_greedy_actions(history o2, history o1, float epsilon){
		// With probability 1-epsilon we do this.
		if (uniform_epsilon_distribution(random_engine) > epsilon){
			// Let PyTorch know that we don't need to keep track of the gradient in this context.
			torch::NoGradGuard no_grad;
			torch::Tensor o2_o1 = torch::cat({o2, o1});
			o2_o1 = o2_o1.to(device);
			return torch::argmax(policy_nets->q_net(o2_o1)).item<int>();
		// With probabiliy epsilon we do this.
		} else {
			// Choose random action for agent 0, return it.
			return uniform_action_distribution(random_engine);
		}
	}

	history POMDP_Agents::get_next_history_2(history o2, action u2, observation z2){
		// Let PyTorch know that we don't need to keep track of the gradient in this context.
		torch::NoGradGuard no_grad;
		// Recast the u2 and z2 as a Tensor and get u2_z2, the entry to the network.
		torch::Tensor u2_z2 = recast_u2_z2(u2, z2);
		// Add batch dimension since RNNs need it.
		o2 = o2.unsqueeze(0);
		// Put it to GPU if needed.
		o2 = o2.to(device);
		// Get next_h0, put it back to CPU (if it was in GPU), and remove the batch dimension.
		return policy_nets->trans_net_2(u2_z2, o2).cpu().squeeze();
	}

	history POMDP_Agents::get_next_history_1(history o1, action u1, observation z1){
		// Let PyTorch know that we don't need to keep track of the gradient in this context.
		torch::NoGradGuard no_grad;
		// Recast the u1 and z1 as a Tensor and get u1_z1, the entry to the network.
		torch::Tensor u1_z1 = recast_u1_z1(u1, z1);
		// Add batch dimension since RNNs need it.
		o1 = o1.unsqueeze(0);
		// Put it to GPU if needed.
		o1 = o1.to(device);
		// Get next_h1, put it back to CPU (if it was in GPU), and remove the batch dimension.
		return policy_nets->trans_net_1(u1_z1, o1).cpu().squeeze();
	}

	torch::Tensor POMDP_Agents::recast_u2_z2(action u2, observation z2){
		// Create one hot vector for u2 with correct number of dimensions.
		torch::Tensor one_hot_u2 = torch::zeros(game->getNumActions(0));
		// Set the correct index to 1, the others stay 0.
		one_hot_u2[u2] = 1;
		// Add batch dimension since RNNs need it.
		one_hot_u2 = one_hot_u2.unsqueeze(0);
		// Put it to GPU if needed.
		one_hot_u2 = one_hot_u2.to(device);
		// Create one hot vector for z2 with correct number of dimensions.
		torch::Tensor one_hot_z2 = torch::zeros(game->getNumObservations(0));
		// Set the correct index to 1, the others stay 0.
		one_hot_z2[z2] = 1;
		// Add batch dimension since RNNs need it.
		one_hot_z2 = one_hot_z2.unsqueeze(0);
		// Put it to GPU if needed.
		one_hot_z2 = one_hot_z2.to(device);
		// Concatonate the two.
		torch::Tensor u2_z2 = torch::cat({one_hot_u2, one_hot_z2}, 1);
		// Return it.
		return u2_z2;
	}

	torch::Tensor POMDP_Agents::recast_u1_z1(action u1, observation z1){
		// Create one hot vector for u1 with correct number of dimensions.
		torch::Tensor one_hot_u1 = torch::zeros(game->getNumActions(1));
		// Set the correct index to 1, the others stay 0.
		one_hot_u1[u1] = 1;
		// Add batch dimension since RNNs need it.
		one_hot_u1 = one_hot_u1.unsqueeze(0);
		// Create one hot vector for z1 with correct number of dimensions.
		torch::Tensor one_hot_z1 = torch::zeros(game->getNumObservations(1));
		// Set the correct index to 1, the others stay 0.
		one_hot_z1[z1] = 1;
		// Add batch dimension since RNNs need it.
		one_hot_z1 = one_hot_z1.unsqueeze(0);
		// Concatonate the two.
		torch::Tensor u1_z1 = torch::cat({one_hot_u1, one_hot_z1}, 1);
		// Put it to GPU if needed.
		u1_z1 = u1_z1.to(device);
		// Return it.
		return u1_z1;
	}

	void POMDP_Agents::update_target_nets(){
		// Create std::stringstream stream.
		std::stringstream stream;
		// Save the parameters of agent 1's policy net into the stream.
		torch::save(policy_nets, stream);
		// Load those weights from the stream into agent 1's target net.
		torch::load(target_nets, stream);
		//
		save_induced_bias();
	}

}
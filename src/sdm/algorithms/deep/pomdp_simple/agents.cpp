#include <sdm/algorithms/deep/pomdp_simple/agents.hpp>

namespace sdm{
	POMDP_Agents_Simple::POMDP_Agents_Simple(
		number q_net_input_dim, number q_net_inner_dim, number q_net_output_dim, 
		std::shared_ptr<sdm::POSG>& game, torch::Device device, float lr, float adam_eps, std::string ib_net_filename
	){
		this->policy_net = DQN(q_net_input_dim, q_net_inner_dim, q_net_output_dim);
		this->policy_net->to(device);

		this->target_net = DQN(q_net_input_dim, q_net_inner_dim, q_net_output_dim);
		this->target_net->to(device);

		this->uniform_epsilon_distribution = std::uniform_real_distribution<double>(0.0, 1.0);
		this->uniform_action_distribution = std::uniform_int_distribution<int>(0, game->getNumActions(0) * game->getNumActions(1) - 1);
		torch::optim::AdamOptions options;
		options.eps(adam_eps);
		options.lr(lr);
		this->optimizer = std::make_shared<torch::optim::Adam>(policy_net->parameters(), options);
		this->device = device;
		this->game = game;
		this->ib_net_filename = ib_net_filename;
		update_target_nets();
	}

	void POMDP_Agents_Simple::save_induced_bias(){
		std::string ib_target_net_filename;
		ib_target_net_filename.append("../models");
		ib_target_net_filename.append("/");
		ib_target_net_filename.append(ib_net_filename);
		ib_target_net_filename.append("_target_net.pt");
		torch::save(policy_net, ib_target_net_filename);
	}

	action POMDP_Agents_Simple::get_epsilon_greedy_actions(history o2, history o1, float epsilon){
		// With probability 1-epsilon we do this.
		if (uniform_epsilon_distribution(random_engine) > epsilon){
			// Let PyTorch know that we don't need to keep track of the gradient in this context.
			torch::NoGradGuard no_grad;
			torch::Tensor o2_o1 = torch::cat({o2, o1});
			o2_o1 = o2_o1.to(device);
			return torch::argmax(policy_net(o2_o1)).item<int>();
		// With probabiliy epsilon we do this.
		} else {
			// Choose random action for agent 0, return it.
			return uniform_action_distribution(random_engine);
		}
	}

	history POMDP_Agents_Simple::get_next_history_2(history o2, action u2, observation z2){
		// torch::Tensor a = torch::tensor({100, 100});
		// torch::Tensor x = torch::tensor({1, 2, 3, 4, 5, 6, 7, 8});
		// std::cout << "x" << std::endl << x << std::endl;
		// x = torch::roll(x, -2);
		// std::cout << "x" << std::endl << x << std::endl;
		// std::cout << "x.slice(0, 0, -2)" << std::endl << x.slice(0, 0, -2) << std::endl;
		// std::cout << "a" << std::endl << a << std::endl;
		// torch::Tensor next_x = torch::cat({x.slice(0, 0, -2), a});
		// std::cout << "next_x" << std::endl << next_x << std::endl;

		// Recast the u2 and z2 as a Tensor and get u2_z2, the entry to the network.
		torch::Tensor u2_z2 = recast_u2_z2(u2, z2);
		u2_z2 = u2_z2.squeeze();
		number u2_z2_size = u2_z2.sizes()[0];
		// std::cout << "o2" << std::endl << o2 << std::endl;
		o2 = torch::roll(o2, -u2_z2_size);
		// std::cout << "o2" << std::endl << o2 << std::endl;
		// std::cout << "o2.slice(0, 0, -u2_z2_size)" << std::endl << o2.slice(0, 0, -u2_z2_size) << std::endl;
		// std::cout << "u2_z2" << std::endl << u2_z2 << std::endl;
		torch::Tensor next_o2 = torch::cat({o2.slice(0, 0, -u2_z2_size), u2_z2});
		// std::cout << "next_o2" << std::endl << next_o2 << std::endl;
		// std::cout << std::endl;
		return next_o2;
	}

	history POMDP_Agents_Simple::get_next_history_1(history o1, action u1, observation z1){
		// Recast the u1 and z1 as a Tensor and get u1_z1, the entry to the network.
		torch::Tensor u1_z1 = recast_u1_z1(u1, z1);
		// torch::roll();
		// // Get next_h0, put it back to CPU (if it was in GPU), and remove the batch dimension.
		// return policy_nets->trans_net_1(u1_z1, o1).cpu().squeeze();
		return o1;///////////
	}

	torch::Tensor POMDP_Agents_Simple::recast_u2_z2(action u2, observation z2){
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

	torch::Tensor POMDP_Agents_Simple::recast_u1_z1(action u1, observation z1){
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

	void POMDP_Agents_Simple::update_target_nets(){
		// Create std::stringstream stream.
		std::stringstream stream;
		// Save the parameters of agent 1's policy net into the stream.
		torch::save(policy_net, stream);
		// Load those weights from the stream into agent 1's target net.
		torch::load(target_net, stream);
		//
		save_induced_bias();
	}

}
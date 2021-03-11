#include <sdm/algorithms/deep/osdpomdp/agents.hpp>

namespace sdm{
	Agents::Agents(
		number agent_1_policy_net_input_dim, number agent_1_policy_net_inner_dim, number agent_1_policy_net_output_dim, 
		std::shared_ptr<sdm::POSG>& game, torch::Device device, float lr, float adam_eps, bool induced_bias, std::string ib_net_filename, number sampling_memory_size
	){
		this->agent_1_policy_net = DQN(agent_1_policy_net_input_dim, agent_1_policy_net_inner_dim, agent_1_policy_net_output_dim);
		this->agent_1_target_net = DQN(agent_1_policy_net_input_dim, agent_1_policy_net_inner_dim, agent_1_policy_net_output_dim);
		// Put the nets the correct device (CPU/GPU).
		this->agent_1_policy_net->to(device);
		this->agent_1_target_net->to(device);

		this->uniform_epsilon_distribution = std::uniform_real_distribution<double>(0.0, 1.0); //

		this->uniform_action_distribution_2 = std::uniform_int_distribution<int>(0, game->getNumActions(0) - 1);
		this->uniform_action_distribution_1 = std::uniform_int_distribution<int>(0, game->getNumActions(1) - 1);

		torch::optim::AdamOptions options;
		options.eps(adam_eps);
		options.lr(lr);
		this->optimizer = std::make_shared<torch::optim::Adam>(agent_1_policy_net->parameters(), options);

		this->device = device;
		
		this->game = game;
		this->sampling_memory_size = sampling_memory_size;
		update_target_net();
		// if (induced_bias){
		// 	number dim_o2 = agent_2_transition_net_hidden_dim;
		// 	number dim_o1 = agent_1_transition_net_hidden_dim;
		// 	// Initialize the induced bias target net with the same dimensions as the policy net of the POMDP.
		// 	this->induced_bias_target_net = DQN(dim_o2 + dim_o1, dim_o2 + dim_o1, game->getNumActions(0) * game->getNumActions(1)); // the 2nd argument is not guranteed to be correct, but normally should be, as of 11.01.2021
		// 	this->induced_bias_target_net->to(device);
		// 	initialize_induced_bias(ib_net_filename);
		// }
	}

	void Agents::initialize_induced_bias(std::string ib_net_filename){
		std::string ib_target_net_filename;
		ib_target_net_filename.append("../models");
		ib_target_net_filename.append("/");
		ib_target_net_filename.append(ib_net_filename);
		ib_target_net_filename.append("_target_net.pt");
		torch::load(induced_bias_target_net, ib_target_net_filename);
	}

	action Agents::get_epsilon_greedy_action_2(history o2, std::vector<history> all_o1s, float epsilon){
		// With probability 1-epsilon we do this.
		if (uniform_epsilon_distribution(random_engine) > epsilon){
			// Let PyTorch know that we don't need to keep track of the gradient in this context.
			torch::NoGradGuard no_grad;
			// Convert all o1s to Tensor.
			torch::Tensor ao1s = torch::cat(all_o1s);
			// Q values for agent 0 (given o2 and u2).
			std::vector<double> q_values;
			// For each u2 possible:
			for(action u2 = 0; u2 < game->getNumActions(0); u2++){
				// Initialize the Q value to 0.
				q_values.push_back(0.0);
				// Create one hot vector for u2 with correct number of dimensions.
				torch::Tensor one_hot_u2 = torch::zeros(game->getNumActions(0));
				// Set the correct index to 1, the others stay 0.
				one_hot_u2[u2] = 1;
				for (int i = 0; i < sampling_memory_size; i++){
					// Extract one of the all o1s.
					history one_o1 = all_o1s[i];
					// Create the Tensor to put in the network.
					torch::Tensor o2_oo1_u2_ao1s = torch::cat({o2, one_o1, one_hot_u2, ao1s}, 0);
					// Put Tensor to GPU if needed.
					o2_oo1_u2_ao1s = o2_oo1_u2_ao1s.to(device);
					// Get the maximum Q value and add it to the relevant u2 index.
					q_values[u2] += torch::max(agent_1_policy_net(o2_oo1_u2_ao1s)).item<double>();
				}
				// Not really needed but just to be correct.
				q_values[u2] = q_values[u2] / sampling_memory_size;
			}
			// Return the argmax.
			return std::distance(q_values.begin(), std::max_element(q_values.begin(), q_values.end()));
		// With probabiliy epsilon we do this.
		} else {
			// Choose random action for agent 0, return it.
			return uniform_action_distribution_2(random_engine);
		}
	}

	action Agents::get_epsilon_greedy_action_1(history o2, history o1, action u2, std::vector<history> all_o1s, float epsilon){
		// With probability 1-epsilon we do this.
		if (uniform_epsilon_distribution(random_engine) > epsilon){
			// Let PyTorch know that we don't need to keep track of the gradient in this context.
			torch::NoGradGuard no_grad;
			// Create one hot vector for u2 with correct number of dimensions.
			torch::Tensor one_hot_u2 = torch::zeros(game->getNumActions(0));
			// Set the correct index to 1, the others stay 0.
			one_hot_u2[u2] = 1;
			// Convert sampled o1s to Tensor.
			torch::Tensor ao1s = torch::cat(all_o1s);
			// Create the Tensor to put in the network.
			torch::Tensor o2_o1_u2_ao1s = torch::cat({o2, o1, one_hot_u2, ao1s}, 0);
			// Put Tensor to GPU if needed.
			o2_o1_u2_ao1s = o2_o1_u2_ao1s.to(device);
			// Put input Tensor to agent 1's policty net, get q values for each possible private action, get the argument which gives the maximum q value, convert it to int.
			// This is agent 1's action. Return it.
			return torch::argmax(agent_1_policy_net(o2_o1_u2_ao1s)).item<int>();
		// With probabiliy epsilon we do this.
		} else {
			// Choose random action for agent 1, return it.
			return uniform_action_distribution_1(random_engine);
		}
	}

	history Agents::get_next_history_2(history o2, action u2, observation z2){
		// Recast the u2 and z2 as a Tensor and get u2_z2, the entry to the network.
		torch::Tensor u2_z2 = recast_u2_z2(u2, z2);
		u2_z2 = u2_z2.squeeze();
		number u2_z2_size = u2_z2.sizes()[0];
		o2 = torch::roll(o2, -u2_z2_size);
		torch::Tensor next_o2 = torch::cat({o2.slice(0, 0, -u2_z2_size), u2_z2});
		return next_o2;
	}

	history Agents::get_next_history_1(history o1, action u1, observation z1){
		// Recast the u1 and z1 as a Tensor and get u1_z1, the entry to the network.
		torch::Tensor u1_z1 = recast_u1_z1(u1, z1);
		u1_z1 = u1_z1.squeeze();
		number u1_z1_size = u1_z1.sizes()[0];
		o1 = torch::roll(o1, -u1_z1_size);
		torch::Tensor next_o1 = torch::cat({o1.slice(0, 0, -u1_z1_size), u1_z1});
		return next_o1;
	}

	torch::Tensor Agents::recast_u2_z2(action u2, observation z2){
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

	torch::Tensor Agents::recast_u1_z1(action u1, observation z1){
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

	void Agents::update_target_net(){
		// Create std::stringstream stream.
		std::stringstream stream;
		// Save the parameters of agent 1's policy net into the stream.
		torch::save(agent_1_policy_net, stream);
		// Load those weights from the stream into agent 1's target net.
		torch::load(agent_1_target_net, stream);
	}
}
#include <sdm/algorithms/deep/osdpomdp/agents.hpp>

namespace sdm{
	Agents::Agents(
		number agent_2_trans_net_input_dim, number agent_2_trans_net_hidden_dim, 
		number agent_1_trans_net_input_dim, number agent_1_trans_net_hidden_dim, 
		number policy_net_input_dim, number policy_net_inner_dim, number policy_net_output_dim, 
		number seed,
		std::shared_ptr<sdm::POSG>& game, torch::Device device, float lr, float adam_eps, bool induced_bias, std::string ib_net_filename, number K
	){
		// Initialize the nets.
		this->agent_2_transition_net = Gated_RNN(agent_2_trans_net_input_dim, agent_2_trans_net_hidden_dim);
		this->agent_1_transition_net = Gated_RNN(agent_1_trans_net_input_dim, agent_1_trans_net_hidden_dim);
		this->policy_net = DQN(policy_net_input_dim, policy_net_inner_dim, policy_net_output_dim);
		this->target_net = DQN(policy_net_input_dim, policy_net_inner_dim, policy_net_output_dim);
		// Put the nets the correct device (CPU/GPU).
		this->agent_2_transition_net->to(device);
		this->agent_1_transition_net->to(device);
		this->policy_net->to(device);
		this->target_net->to(device);
		// Initialize random random ditributions.
		this->uniform_epsilon_distribution = std::uniform_real_distribution<double>(0.0, 1.0);
		this->uniform_action_distribution_2 = std::uniform_int_distribution<int>(0, game->getNumActions(0) - 1);
		this->uniform_action_distribution_1 = std::uniform_int_distribution<int>(0, game->getNumActions(1) - 1);
		this->random_engine.seed(seed);
		//Initialize Adam and its options.
		torch::optim::AdamOptions options;
		options.eps(adam_eps);
		options.lr(lr);
		this->optimizer = std::make_shared<torch::optim::Adam>(policy_net->parameters(), options);

		this->device = device;		
		this->game = game;
		this->K = K;
		
		update_target_net();
		// if (induced_bias){
		// 	number dim_o2 = agent_2_trans_net_hidden_dim;
		// 	number dim_o1 = agent_1_trans_net_hidden_dim;
		// 	// Initialize the induced bias target net with the same dimensions as the policy net of the POMDP.
		// 	this->induced_bias_target_net = DQN(dim_o2 + dim_o1, dim_o2 + dim_o1, game->getNumActions(0) * game->getNumActions(1)); // the 2nd argument is not guranteed to be correct, but normally should be, as of 11.01.2021
		// 	this->induced_bias_target_net->to(device);
		// 	initialize_induced_bias(ib_net_filename);
		// }
	}

	void Agents::initialize_induced_bias(std::string ib_net_filename){
		////////////
		////////////
		std::string ib_target_net_filename;
		ib_target_net_filename.append("../models");
		ib_target_net_filename.append("/");
		ib_target_net_filename.append(ib_net_filename);
		ib_target_net_filename.append("_target_net.pt");
		torch::load(induced_bias_target_net, ib_target_net_filename);
	}
	// u_{\tau}^{2} \overset{extract}{\leftarrow}  \arg \max_{{{u}'}_{\tau}} \sum_{{k}'=0}^{|M|-1}PolicyNet(o_{\tau}^{2}, o_{\tau}^{1, ({k}')} | o_{\tau}^{2}, \{o_{\tau}^{1,(k)} | o_{\tau}^{2} \}_{k=0}^{|M|-1}, Pr\{x_{\tau}| o_{\tau}^{2}\}, {{u}'}_{\tau})
	action Agents::get_greedy_action_2(history o2, std::vector<history> o1s){
		// Let PyTorch know that we don't need to keep track of the gradient in this context.
		torch::NoGradGuard no_grad;
		// Q values for agent 0 (given o2 and u2).
		std::vector<double> valid_q_values;
		// For each u2 possible:
		for(action u2 = 0; u2 < game->getNumActions(0); u2++){
			// Initialize the Q value to 0.
			valid_q_values.push_back(0.0);
			for (int k = 0; k < K; k++){
				// Extract one of the all o1s.
				history o1 = o1s[k];
				// Create the Tensor to put in the network.
				torch::Tensor o2_o1_o1s = torch::cat({o2, o1, torch::cat(o1s)}, 0);
				// Put Tensor to GPU if needed.
				o2_o1_o1s = o2_o1_o1s.to(device);
				// Get u1, given o2, o1, u2, o1s.
				action u1 = get_greedy_action_1(o2, o1, u2, o1s);
				// Get joint action u given u2 and u1.
				action u = game->getActionSpace().joint2single({u2, u1});
				// Get the Q value given u and add it to the relevant u2 index.
				valid_q_values[u2] += policy_net(o2_o1_o1s).index({u}).item<double>();
			}
		}
		// Return the argmax.
		return std::distance(valid_q_values.begin(), std::max_element(valid_q_values.begin(), valid_q_values.end()));
	}
	// u_{\tau}^{1} = \arg \max_{{{u}'}_{\tau}^{1}} PolicyNet(o_{\tau}^{2}, o_{\tau}^{1}, \{o_{\tau}^{1,(k)} | o_{\tau}^{2} \}_{k=0}^{|M|-1}, Pr\{x_{\tau}| o_{\tau}^{2}\}, {u}_{\tau}^{2}, {{u}'}_{\tau}^{1})
	action Agents::get_greedy_action_1(history o2, history o1, action u2, std::vector<history> o1s){
		// Let PyTorch know that we don't need to keep track of the gradient in this context.
		torch::NoGradGuard no_grad;
		// Create the Tensor to put in the network.
		torch::Tensor o2_o1_o1s = torch::cat({o2, o1, torch::cat(o1s)}, 0);
		// Put Tensor to GPU if needed.
		o2_o1_o1s = o2_o1_o1s.to(device);
		// Put input Tensor to policy net, get q values for each possible joint action.
		torch::Tensor q_values = policy_net(o2_o1_o1s);
		// Cretae vector for valid q values, meaning q values that are possible given u2.
		std::vector<double> valid_q_values = {};
		// For each possible u1:
		for(action u1 = 0; u1 < game->getNumActions(1); u1++){
			action u = game->getActionSpace().joint2single({u2, u1});
			// Add the corresponding q value given u.
			valid_q_values.push_back(q_values.index({u}).item<double>());
		}
		// Return the argmax of valid q values.
		return std::distance(valid_q_values.begin(), std::max_element(valid_q_values.begin(), valid_q_values.end()));
	}
	//
	action Agents::get_epsilon_greedy_action_2(history o2, std::vector<history> all_o1s, float epsilon){
		// With probability 1-epsilon we do this.
		if (uniform_epsilon_distribution(random_engine) > epsilon){
			// Get greedy action.
			return get_greedy_action_2(o2, all_o1s);
		// With probabiliy epsilon we do this.
		} else {
			// Get random action.
			return uniform_action_distribution_2(random_engine);
		}
	}
	//
	action Agents::get_epsilon_greedy_action_1(history o2, history o1, action u2, std::vector<history> all_o1s, float epsilon){
		// With probability 1-epsilon we do this.
		if (uniform_epsilon_distribution(random_engine) > epsilon){
			// Get greedy action.
			return get_greedy_action_1(o2, o1, u2, all_o1s);
		// With probabiliy epsilon we do this.
		} else {
			// Get random action.
			return uniform_action_distribution_1(random_engine);
		}
	}

	history Agents::get_next_history_2(history o2, action u2, observation z2){
		// Let PyTorch know that we don't need to keep track of the gradient in this context.
		torch::NoGradGuard no_grad;
		// Recast the u2 and z2 as a Tensor and get u2_z2, the entry to the network.
		torch::Tensor u2_z2 = recast_u2_z2(u2, z2);
		// Add batch dimension since RNNs need it.
		o2 = o2.unsqueeze(0);
		// Put it to GPU if needed.
		o2 = o2.to(device);
		// Get next_h0, put it back to CPU (if it was in GPU), and remove the batch dimension.
		return agent_2_transition_net(u2_z2, o2).cpu().squeeze();
	}

	history Agents::get_next_history_1(history o1, action u1, observation z1){
		// Let PyTorch know that we don't need to keep track of the gradient in this context.
		torch::NoGradGuard no_grad;
		// Recast the u1 and z1 as a Tensor and get u1_z1, the entry to the network.
		torch::Tensor u1_z1 = recast_u1_z1(u1, z1);
		// Add batch dimension since RNNs need it.
		o1 = o1.unsqueeze(0);
		// Put it to GPU if needed.
		o1 = o1.to(device);
		// Get next_h1, put it back to CPU (if it was in GPU), and remove the batch dimension.
		return agent_1_transition_net(u1_z1, o1).cpu().squeeze();
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
		torch::save(policy_net, stream);
		// Load those weights from the stream into agent 1's target net.
		torch::load(target_net, stream);
	}
}
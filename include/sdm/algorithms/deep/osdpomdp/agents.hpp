#pragma once

#include <random>
#include <math.h>
#include <vector>
#include <iterator>
#include <experimental/algorithm>
#include <tuple>
#include <utility>

#include <boost/program_options.hpp>
#include <boost/any.hpp>

#include <torch/torch.h>

#include <sdm/worlds.hpp>
#include <sdm/algorithms/deep/dnn.hpp>

namespace sdm{
	struct Agents {
		// The game to be solved.
		std::shared_ptr<sdm::POSG> game;
		//
		number sampling_memory_size;
		// Initialize the Agents.
		Agents(number, number, number, number, number, number, std::shared_ptr<sdm::POSG>&, torch::Device, float, float, bool, std::string, number);
		// C++ random number engine.
		std::default_random_engine random_engine;
		// Uniform epsilon distribution, returns random double between 0 and 1.
		std::uniform_real_distribution<double> uniform_epsilon_distribution;
		// Uniform action distribution for agent 2, returns random int between 0 and nA(0)-1.
		std::uniform_int_distribution<int> uniform_action_distribution_2;
		// Uniform action distribution for agent 1, returns random int between 0 and nA(1)-1.
		std::uniform_int_distribution<int> uniform_action_distribution_1;
		// Optimizer.
		std::shared_ptr<torch::optim::Adam> optimizer;
		// Policy net of agent 1.
		DQN agent_1_policy_net{nullptr};

		RNN agent_2_transition_net{nullptr};

		RNN agent_1_transition_net{nullptr};
		// The Target Network is a delayed copy of the Policy Network of Agent 1. Its purpose is to not have moving targets during update of networks.
		DQN agent_1_target_net{nullptr};
		// The policy_net of the POMDP. 
		DQN induced_bias_target_net{nullptr};
		// CPU or GPU.
		torch::Device device = torch::Device(torch::kCPU);
		// Given history o2 and histories o1s get epsilon-greedy action u2.
		action get_epsilon_greedy_action_2(history, std::vector<history>, float);
		// Given history o2, history o1, action u2, histories o1s; get epsilon-greedy action u1.
		action get_epsilon_greedy_action_1(history, history, action, std::vector<history>, float);
		// Given history o2, action u2, observation z2; get next history next_o2.
		history get_next_history_2(history, action, observation);
		// Given history o1, action u1, observation z1; get next history next_o1.
		history get_next_history_1(history, action, observation);
		// Helper function to create tensor u2_z2 from action u2 and observation u2.
		torch::Tensor recast_u2_z2(action, observation);
		// Helper function to create tensor u1_z1 from action u1 and observation o1.
		torch::Tensor recast_u1_z1(action, observation);
		// Update the target network by copying the parameters of agent 1's policy network into the target networks parameters.
		void update_target_net();
		// Load the history transition nets that were saved after the POMDP was solved. Also load the induced_bias_target_net which was the policy_net for the POMDP.
		void initialize_induced_bias(std::string);
	};
}

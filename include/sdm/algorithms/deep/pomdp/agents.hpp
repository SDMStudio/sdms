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
	struct POMDP_Agents {
		// The game to be solved.
		std::shared_ptr<sdm::POSG> game;
		// The name of the file that the networks will be ssaved as. These can later be used for the solution of the OSDPOMDP as induced bias.
		std::string ib_net_filename;
		// Initialize the Agents.
		POMDP_Agents(number, number, number, number, number, number, number, std::shared_ptr<sdm::POSG>&, torch::Device, float, float, std::string);
		// C++ random number engine.
		std::default_random_engine random_engine;
		// Uniform epsilon distribution, returns random double between 0 and 1.
		std::uniform_real_distribution<double> uniform_epsilon_distribution;
		// Uniform action distribution for agent 2, returns random int between 0 and nA(0)*nA(1)-1.
		std::uniform_int_distribution<int> uniform_action_distribution;
		// Optimizer.
		std::shared_ptr<torch::optim::Adam> optimizer;
		// DRQN that is used to act.
		DRQN policy_nets{nullptr};
		// DRQN that is used to get the target Q Values during updates.
		DRQN target_nets{nullptr};
		// CPU or GPU.
		torch::Device device = torch::Device(torch::kCPU);
		// Given histories o2 and o1s, get epsilon-greedy action u2_u1.
		action get_epsilon_greedy_actions(history, history, float);
		// Given history o2, action u2, observation z2; get next history next_o2.
		history get_next_history_2(history, action, observation);
		// Given history o1, action u1, observation z1; get next history next_o1.
		history get_next_history_1(history, action, observation);
		// Helper function to create tensor u2_z2 from action u2 and observation u2.
		torch::Tensor recast_u2_z2(action, observation);
		// Helper function to create tensor u1_z1 from action u1 and observation u1.
		torch::Tensor recast_u1_z1(action, observation);
		// Update the target nets by copying the parameters of policy nets into the target networks' parameters.
		void update_target_nets();
		// Save the nets' parameters so that they can later be used to solve the OSDPOMDP as induced bias.
		void save_induced_bias();
	};
}
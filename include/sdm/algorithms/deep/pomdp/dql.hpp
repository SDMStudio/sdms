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
#include <sdm/algorithms/deep/pomdp/agents.hpp>
#include <sdm/algorithms/deep/pomdp/replay_memory.hpp>
#include <sdm/algorithms/deep/pomdp/models_update_rules.hpp>

namespace sdm{
	struct DQL {
		// Game to be solved.
		std::shared_ptr<sdm::POSG> game;
		// Current episode.
		int episode;
		// Total number of epidoes.
		int episodes;
		// The number time steps that the gradient will propagate during Back Propagation Through Time (BPTT).
		number tao;
		// Number of transition sequences that will be sampled from each episode for training
		number eta;
		// Horizon of the game to solve.
		number horizon;
		// Batch size for updating the models.
		number batch_size;
		// Dimension of history of agent 2.
		number dim_o2;
		// Dimension of history of agent 2.
		number dim_o1;
		// How many episodes go before we update the target net.
		number target_update;
		// Interval for printing the relevant statistics.
		number print_every;
		// Current step.
		number step;
		// Final value of epsilon.
		float eps_end;
		// Initial value of epsilon.
		float eps_start;
		// Decay rate of epsilon.
		float eps_decay;
		// Discount factor of the game i.e. the degree to which the next reward is less imporant than the current one.
		float discount_factor;
		// This lets the E[R] be more smooth. A value between [0, 1). 0: Ignore previous Rs. .9: It's as if it's the average of the last 10 Rs etc.
		float rolling_factor;
		// Learning rate.
		float lr;
		// Epsilon for adam optimizer.
		float adam_eps;
		// The epsilon.
		float epsilon;
		// Discount factor (it looks nicer this way)
		float GAMMA;
		// Q value loss that is returned from the update function.
		double q_value_loss;
		// Discounted cumulative reward at the end of the episode. 
		reward R;
		// E[R]
		reward E_R;
		// Reward for the current step.
		reward r;
		// Action of agent 2.
		action u2;
		// Action of agent 1.
		action u1;
		// The joint action of agents 2 and 1. Needed later.
		action u2_u1;
		// Current state of the game.
		state x;
		// Next state of the game.
		state next_x;
		// Observation of agent 2.
		observation z2;
		// Observation of agent 1.
		observation z1;
		// History of agent 2.
		history o2;
		// Next history of agent 2.
		history next_o2;
		// History of agent 1.
		history o1;
		// Next history of agent 1.
		history next_o1;
		// For updating the models.
		std::shared_ptr<POMDP_ModelsUpdateRules> models_update_rules;
		// Experience replay memory.
		std::shared_ptr<POMDP_ReplayMemory> replay_memory;
		// Agents object for agents 2 and 1.
		std::shared_ptr<POMDP_Agents> agents;
		// Number of steps done in total.
		int steps_done;
		// Initialize the method for solving the problem.
		DQL(
			int, 
			number, number, number, number, number, number, number, number, number, 
			float, float, float, float, float, float, float, 
			torch::Device, std::shared_ptr<sdm::POSG>&, int, std::string, bool
		);
		// (Should be in POSG class.) Used to get the joint action a from private actions u2 and u1.
		action get_u_from_u2_u1(action, action);
		// Update epsilon according to the specified decay rate.
		void update_epsilon();
		// (Should be in POSG class.) Using action u2 and action u1; get observation z2, observation z1, and reward r.
		std::tuple<observation, observation, reward> act();
		// Solve the problem.
		void solve();
		// Update history o2, histories o1s, epsilon, and target network.
		void end_step();
		// Log episode, epsilon, q value loss, and E[R].
		void end_episode();
		// Initialize the problem.
		void initialize();
		// Initialize everything that will be used: R, q value loses, x, next_x, o2, next_o2, o1, next_o1, u2, u1, z2, z1, r, sampled_xs, next_sampled_xs, sampled_s1s, nsampled_s1s
		void initialize_episode();
		// Use the models update rules to update the models.
		void update_models();
		// Add current transition to the replay memory.
		void update_replay_memory();
	};
}
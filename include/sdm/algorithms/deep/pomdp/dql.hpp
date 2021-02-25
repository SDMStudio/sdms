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

		//
		number tao;
		number horizon, batch_size, dim_o2, dim_o1, target_update, print_every, step;
		float eps_end, eps_start, eps_decay, discount_factor, rolling_factor, lr, adam_eps, epsilon, GAMMA;
		double q_value_loss;
		// Discounted cumulative reward at the end of the episode. 
		reward R;
		// E[R]
		reward E_R;
		reward r;
		action u2, u1, u2_u1;
		state x, next_x;
		observation z1, z2;
		history o2, next_o2, o1, next_o1;
		std::shared_ptr<POMDP_ModelsUpdateRules> models_update_rules;
		std::shared_ptr<POMDP_ReplayMemory> replay_memory;
		std::shared_ptr<POMDP_Agents> agents;
		// Number of steps done in total.
		int steps_done;
		// Initialize the method for solving the problem.
		DQL(
			int, 
			number, number, number, number, number, number, number,
			float, float, float, float, float, float, float, 
			torch::Device, std::shared_ptr<sdm::POSG>&, int, std::string
		);
		// (Should be in POSG class.) Used to get the joint action a from private actions u2 and u1.
		action get_a_from_u2_u1(action, action);
		// For now it just sets E[R] to 0.
		void estimate_initial_E_R();
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
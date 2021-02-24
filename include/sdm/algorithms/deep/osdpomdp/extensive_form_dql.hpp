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
#include <sdm/algorithms/deep/osdpomdp/agents.hpp>
#include <sdm/algorithms/deep/osdpomdp/replay_memory.hpp>
#include <sdm/algorithms/deep/osdpomdp/models_update_rules.hpp>

namespace sdm{
	struct ExtensiveFormDQL {
		//
		std::ofstream output_file;
		// The game to be solved.
		std::shared_ptr<sdm::POSG> game;
		// Current episode.
		int episode;
		// Total number of epidoes.
		int episodes;
		// Indice of parallel world, used to collect more data.
		number i;
		// Indice of which sampling world we're in.
		number m;
		// The world that choose to follow from the previous step.
		number m_star;
		number horizon, batch_size, i_batch_size, dim_o2, dim_o1, target_update, dim_i1, sampling_memory_size, print_every, step;
		float eps_end, eps_start, eps_decay, discount_factor, rolling_factor, lr, adam_eps, epsilon, GAMMA, alpha_decay, alpha;
		double q_value_loss;
		// Do we use induced bias i.e. the solution of the POMDP or not?
		bool induced_bias;
		// C++ random number engine.
		std::default_random_engine random_engine;
		// Uniform m distribution, returns random int between 0 and M-1.
		std::uniform_int_distribution<int> uniform_m_distribution;
		// Discounted cumulative reward at the end of the episode. 
		reward R;
		// E[R]
		reward E_R;
		std::vector<std::vector<reward>> rs;
		// std::vector<history> o2, next_o2;
		std::vector<action> u2;
		std::vector<std::vector<state>> xs, next_xs;
		std::vector<std::vector<action>> u1s;
		std::vector<std::vector<observation>> z1s, z2s;
		std::vector<std::vector<history>> o2s, next_o2s, o1s, next_o1s;
		std::shared_ptr<ModelsUpdateRules> models_update_rules;
		std::shared_ptr<ReplayMemory> replay_memory;
		std::shared_ptr<Agents> agents;
		// Number of steps done in total.
		int steps_done;
		// Initialize the method for solving the problem.
		ExtensiveFormDQL(
			int, 
			number, number, number, number, number, number, number, number, number, 
			float, float, float, float, float, float, float, float, 
			torch::Device, std::shared_ptr<sdm::POSG>&, int, std::string, bool, std::string
		);
		// (Should be in POSG class.) Used to get the joint action a from private actions u2 and u1.
		action get_a_from_u2_u1(action, action);
		// For now it just sets E[R] to 0.
		void estimate_initial_E_R();
		// Update epsilon according to the specified decay rate.
		void update_epsilon();
		// Update alpha according to the specified decay rate.
		void update_alpha();
		// Helper function to initiate xs.
		std::vector<state> initiate_xs();
		// Helper function to initiate u1s.
		std::vector<action> initiate_u1s();
		// Helper function to initiate z1s.
		std::vector<observation> initiate_z1s();
		// Helper function to initiate o1s.
		std::vector<history> initiate_o1s();
		// Helper function to initiate rs.
		std::vector<reward> initiate_rs();
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
		//
		void determine_branch();
	};
}
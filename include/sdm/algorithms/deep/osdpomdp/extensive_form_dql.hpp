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
		// The game to be solved.
		std::shared_ptr<sdm::POSG> game;
		// Current episode.
		int episode;
		// Total number of epidoes.
		int episodes;
		// Indice of which sampling world we're in.
		number k;
		// Size of the history of agent 2.
		number dim_o2;
		// Size of the history of agent 1.
		number dim_o1;
		// Horizon of the game to solve.
		number horizon;
		// Batch size for updating the models.
		number batch_size;
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
		// This lets the E[R] be more smooth. A value between [0, 1). 0: Ignore previous Rs. .9: It's as if it's the average of the last 10 Rs etc.
		float rolling_factor;
		// Learning rate.
		float lr;
		// Epsilon for adam optimizer.
		float adam_eps;
		// The epsilon.
		float epsilon;
		//  Discount factor of the game i.e. the degree to which the next reward is less imporant than the current one.
		float GAMMA;
		// Inner dimension of DQNs (policy net and target net).
		number dim_i;
		// How many sampling/parallel worlds there will be for agent 1 to interact with the environment?
		number K;
		// How much of IB solution will be used as the target during updates.
		float alpha;
		// Decay rate of alpha.
		float alpha_decay;
		// Q value loss that is returned from the update function.
		double q_value_loss;
		// Do we use induced bias i.e. the solution of the POMDP or not?
		bool induced_bias;
		// C++ random number engine.
		std::default_random_engine random_engine;
		// Uniform m distribution, returns random int between 0 and M-1.
		std::uniform_int_distribution<int> uniform_m_distribution;
		// Uniform m distribution, returns random int between 0 and nZ(0)-1.
		std::uniform_int_distribution<int> uniform_z2_distribution;
		// Discounted cumulative reward. 
		reward R;
		// E[R]
		reward E_R;
		// Rewards for the current steps, one for each sampling/parallel world.
		std::vector<reward> rs;
		// Action of agent 2.
		action u2;
		// Actions of agent 1, one for each sampling/parallel world.
		std::vector<action> u1s;
		// Current states of the game, one for each sampling/parallel world.
		std::vector<state> xs;
		// Next states of the game, one for each sampling/parallel world.
		std::vector<state> next_xs;
		// Observation of agent 2.
		observation z2;
		// A possible observation of agent 2, it should be equal to z2.
		observation candidate_z2;
		// Observations of agent 1, one for each sampling/parallel world.
		std::vector<observation> z1s;
		// History of agent 2.
		history o2;
		// Next history of agent 2.
		history next_o2;
		// Histories of agent 1, one for each sampling/parallel world.
		std::vector<history> o1s;
		// Next histories of agent 1, one for each sampling/parallel world.
		std::vector<history> next_o1s;
		// For updating the models.
		std::shared_ptr<ModelsUpdateRules> models_update_rules;
		// Experience replay memory.
		std::shared_ptr<ReplayMemory> replay_memory;
		// Agents object for agents 2 and 1.
		std::shared_ptr<Agents> agents;
		// Number of steps done in total.
		int steps_done;
		// Initialize the method for solving the problem.
		ExtensiveFormDQL(
			int, 
			number, number, number, number, number, number, number, number, number, 
			float, float, float, float, float, float, float, 
			torch::Device, std::shared_ptr<sdm::POSG>&, int, bool, std::string
		);
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
		// (Should be in POSG class.) Using action u2 and action u1; get observation z2, observation z1, and reward r. The states xs are used and states next_xs are updated.
		std::tuple<observation, observation, reward> act();
		// Solve the problem.
		void solve();
		// Update history o2, histories o1s, epsilon, alpha.
		void end_step();
		// Log episode, epsilon, q value loss, and E[R]. Also update target net.
		void end_episode();
		// Initialize the problem.
		void initialize();
		// Initialize everything that will be used during the episode: R, q value loses, x, next_x, o2, next_o2, o1, next_o1, u2, u1, z2, z1, r, sampled_xs, next_sampled_xs, sampled_s1s, nsampled_s1s
		void initialize_episode();
		// Use the models update rules to update the models.
		void update_models();
		// Add current transition to the replay memory.
		void update_replay_memory();
	};
}
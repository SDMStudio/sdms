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

#include <sdm/types.hpp>

namespace sdm{
	struct POMDP_ReplayMemory{
		// Capacity of the replay memory, i.e. how many transitions it can store.
		int capacity;
		// The number time steps that the gradient will propagate during Back Propagation Through Time (BPTT).
		number tao;
		// Number of transition sequences that will be sampled from each episode for training
		number eta;
		// Batch size of updates i.e. how many transitions at a time we use to update the parameters of the nets.
		number batch_size;
		// Vector of vector of transitions that are stored in the replay memory.
		std::vector<pomdp_transitions_sequence> memory;
		// The current episode tht is being filled. This doesn't get added to the memory till it's complete since it'd pose problems during sampling.
		pomdp_transitions_sequence current_episode_transitions;
		// Horizon of the game to solve.
		number horizon;
		// C++ random number engine.
		std::default_random_engine random_engine;
		// Gives a random number between 0 and horizon - tao that is used for the starting index.
		std::uniform_int_distribution<int> uniform_transitions_distribution;
		// Construct the replay memory with given capacity, tao, horizon, batch_size.
		POMDP_ReplayMemory(int, number, number, number, number, number);
		// Push the transition into the replay memory given the episode and the step.
		void push(pomdp_transition, int, number);
		// Sample the replay memory and get transitions sequences.
		std::vector<pomdp_transitions_sequence> sample();
		// Return the current size of the replay memory, i.e. how many transition it has at the moment.
		int size();
	};
}
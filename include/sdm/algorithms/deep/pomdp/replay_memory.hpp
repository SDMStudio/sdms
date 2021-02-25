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
		//
		number tao;
		// Vector of vector of transitions that are stored in the replay memory.
		std::vector<pomdp_recurrent_transitions> memory;
		//
		pomdp_recurrent_transitions current_episode_transitions;
		// 
		number horizon;
		// C++ random number engine.
		std::default_random_engine random_engine;
		//
		std::uniform_int_distribution<int> uniform_transitions_distribution;
		// Construct the replay memory with given capacity.
		POMDP_ReplayMemory(int, number, number);
		// Push the transition into the replay memory.
		void push(pomdp_transition, int, number);
		// Sample the given number of transitions at random from the replay memory.
		std::vector<pomdp_recurrent_transitions> sample(int);
		// Return the current size of the replay memory, i.e. how many transition it has at the moment.
		int size();
	};
}
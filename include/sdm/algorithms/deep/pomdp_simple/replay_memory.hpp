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
	struct POMDP_ReplayMemory_Simple{
		// Capacity of the replay memory, i.e. how many transitions it can store.
		int capacity;
		// Batch size of updates i.e. how many transitions at a time we use to update the parameters of the nets.
		number batch_size;
		// Vector of transitions that are stored in the replay memory.
		std::vector<pomdp_transition_simple> memory;
		// Position of the next transition to be added. The implementation is such that when it overflows the capacity, the transition to be added is inserted in a cyclic fashion to override a previous one.
		int position;
		// C++ random number engine.
		std::default_random_engine random_engine;
		// Construct the replay memory with given capacity.
		POMDP_ReplayMemory_Simple(int, number, number);
		// Push the transition into the replay memory.
		void push(pomdp_transition_simple);
		// Sample transitions at random from the replay memory.
		std::vector<pomdp_transition_simple> sample();
		// Return the current size of the replay memory, i.e. how many transition it has at the moment.
		int size();
	};
}
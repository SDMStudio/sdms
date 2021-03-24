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
	struct ReplayMemory{
		// Capacity of the replay memory, i.e. how many transitions it can store.
		int capacity;
		// Vector of transitions that are stored in the replay memory.
		std::vector<transition> memory;
		// Position of the next transition to be added. The implementation is such that when it overflows the capacity, the transition to be added is inserted in a cyclic fashion to override a previous one.
		int position;
		// C++ random number engine.
		std::default_random_engine random_engine;
		// Construct the replay memory with given capacity.
		ReplayMemory(int, number);
		// Push the transition into the replay memory.
		void push(transition);
		// Sample the given number of transitions at random from the replay memory.
		std::vector<transition> sample(int);
		// Return the current size of the replay memory, i.e. how many transition it has at the moment.
		int size();
	};
}
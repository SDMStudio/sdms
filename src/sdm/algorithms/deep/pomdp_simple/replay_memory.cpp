#include <sdm/algorithms/deep/pomdp_simple/replay_memory.hpp>

namespace sdm{
	POMDP_ReplayMemory_Simple::POMDP_ReplayMemory_Simple(int capacity, number batch_size){
		// Set the capacity of the replay memory.
		this->capacity = capacity;
		//
		this->batch_size = batch_size;
		// Initialize the position.
		this->position = 0;
	}

	void POMDP_ReplayMemory_Simple::push(pomdp_transition_simple t){
		// If there are less transitions than the capacity of the replay memory:
		if (this->memory.size() < this->capacity) {
			// Create an empty sampled hidden history 1's vector.
			std::vector<history> sh1s = {torch::zeros(1), torch::zeros(1)};
			// Push back an empty transition to the replay memory.
			this->memory.push_back(std::make_tuple(torch::zeros(1), torch::zeros(1), 0, 0, torch::zeros(1), torch::zeros(1)));
		}
		// Replace the transition at position with the given transition.
		this->memory[this->position] = t;
		// Update the position in a cycling manner with respect to the capacity.
		this->position = (this->position + 1) % this->capacity;
	}

	std::vector<pomdp_transition_simple> POMDP_ReplayMemory_Simple::sample(){
		// Initialize the transitions std::vector to be returned.
		std::vector<pomdp_transition_simple> transitions;
		// Sample from the memory according to Mersenne Twister pseudorandom number generator and fill the transitions std::vector.
		std::experimental::sample(this->memory.begin(), this->memory.end(), std::back_inserter(transitions), batch_size, std::mt19937{std::random_device{}()});
		// Return transitions.
		return transitions;
	}

	int POMDP_ReplayMemory_Simple::size(){
		// Return the of the memory of the replay memory i.e. number of transition stored.
		return this->memory.size();
	}

}
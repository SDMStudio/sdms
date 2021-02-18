#include <sdm/algorithms/deep/osdpomdp/replay_memory.hpp>


namespace sdm{
	ReplayMemory::ReplayMemory(int capacity){
		// Set the capacity of the replay memory.
		this->capacity = capacity;
		// Initialize the position.
		this->position = 0;
	}

	void ReplayMemory::push(transition t){
		// If there are less transitions than the capacity of the replay memory:
		if (this->memory.size() < this->capacity) {
			// Create an empty sampled hidden history 1's vector.
			std::vector<history> sh1s = {torch::zeros(1), torch::zeros(1)};
			// Push back an empty transition to the replay memory.
			this->memory.push_back(std::make_tuple(torch::zeros(1), torch::zeros(1), sh1s, 0, 0, 0, torch::zeros(1), torch::zeros(1), sh1s));
		}
		// Replace the transition at position with the given transition.
		this->memory[this->position] = t;
		// Update the position in a cycling manner with respect to the capacity.
		this->position = (this->position + 1) % this->capacity;
	}

	// ReplayMemory \overset{Sample}{\rightarrow}o_{t}^{0}, o_{t}^{1}, \{o_{t}^{1}\}, a_{t}^{0}, a_{t}^{1}, z_{t}^{0}, z_{t}^{1}, r_{t}
	std::vector<transition> ReplayMemory::sample(int batch_size){
		// Initialize the transitions std::vector to be returned.
		std::vector<transition> transitions;
		// Sample from the memory according to Mersenne Twister pseudorandom number generator and fill the transitions std::vector.
		std::experimental::sample(this->memory.begin(), this->memory.end(), std::back_inserter(transitions), batch_size, std::mt19937{std::random_device{}()});
		// Return transitions.
		return transitions;
	}

	int ReplayMemory::size(){
		// Return the of the memory of the replay memory i.e. number of transition stored.
		return this->memory.size();
	}
}
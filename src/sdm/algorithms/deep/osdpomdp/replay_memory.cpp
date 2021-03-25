#include <sdm/algorithms/deep/osdpomdp/replay_memory.hpp>


namespace sdm{
	ReplayMemory::ReplayMemory(int capacity, number seed){
		// Set the capacity of the replay memory.
		this->capacity = capacity;
		// Initialize the position.
		this->position = 0;
		//
		this->random_engine.seed(seed);
	}

	void ReplayMemory::push(transition t){
		// If there are less transitions than the capacity of the replay memory:
		if (memory.size() < capacity) {
			// Create an empty agent 1 histories vector.
			std::vector<history> o1s = {torch::zeros(1)};
			// Push back an empty transition to the replay memory.
			memory.push_back(std::make_tuple(torch::zeros(1), torch::zeros(1), o1s, 0, 0, torch::zeros(1), torch::zeros(1), o1s));
		}
		// Replace the transition at position with the given transition.
		memory[position] = t;
		// Update the position in a cycling manner with respect to the capacity.
		position = (position + 1) % capacity;
	}

	// ReplayMemory \overset{Sample}{\rightarrow} (o_{\tau}^{2}, o_{\tau}^{1}, \{o_{\tau}^{1,(m)} | o_{\tau}^{2} \}_{m=0}^{|M|-1}, Pr\{x_{\tau}| o_{\tau}^{2}\}, u_{\tau}, r_{\tau}, o_{\tau+1}^{2}, o_{\tau+1}^{1}, \{o_{\tau+1}^{1,(m)} | o_{\tau+1}^{2} \}_{m=0}^{|M|-1}, Pr\{x_{\tau+1}| o_{\tau+1}^{2}\})
	std::vector<transition> ReplayMemory::sample(int batch_size){
		// Initialize the transitions std::vector to be returned.
		std::vector<transition> transitions;
		// Sample from the memory according to Mersenne Twister pseudorandom number generator and fill the transitions std::vector.
		std::experimental::sample(memory.begin(), memory.end(), std::back_inserter(transitions), batch_size, random_engine);
		// Return transitions.
		return transitions;
	}

	int ReplayMemory::size(){
		// Return the of the memory of the replay memory i.e. number of transition stored.
		return memory.size();
	}
}
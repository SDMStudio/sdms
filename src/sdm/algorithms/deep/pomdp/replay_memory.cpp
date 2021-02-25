#include <sdm/algorithms/deep/pomdp/replay_memory.hpp>

namespace sdm{
	POMDP_ReplayMemory::POMDP_ReplayMemory(int capacity, number tao, number horizon){
		// Set the capacity of the replay memory.
		this->capacity = capacity;
		// Initialize the horizon.
		this->horizon = horizon;
		//
		this->tao = tao;
		//
		this->uniform_transitions_distribution = std::uniform_int_distribution<int>(0, horizon - tao);
	}

	void POMDP_ReplayMemory::push(pomdp_transition t, int episode, number step){
		// Add the current transition to the
		current_episode_transitions.push_back(t);
		//
		if (step == horizon - 1){
			//
			pomdp_recurrent_transitions new_episode_transitions = current_episode_transitions;
			//
			if (memory.size() < capacity){
				//
				memory.push_back(new_episode_transitions);
			//
			} else {
				//
				memory[episode % capacity] = new_episode_transitions;
			}
			//
			current_episode_transitions.clear();
		}
	}

	//
	std::vector<pomdp_recurrent_transitions> POMDP_ReplayMemory::sample(int batch_size){
		// Initialize the transitions to be returned.
		std::vector<pomdp_recurrent_transitions> recurrent_transitions_vector(tao);
		// Episodes to be randomly selected from the experience replay memory.
		std::vector<pomdp_recurrent_transitions> episodes_selected;
		// Sample from the memory according to Mersenne Twister pseudorandom number generator and fill the episodes_selected std::vector.
		std::experimental::sample(memory.begin(), memory.end(), std::back_inserter(episodes_selected), batch_size, std::mt19937{std::random_device{}()});

		//
		for (int i = 0; i < batch_size; i++){
			//
			int starting_indice = uniform_transitions_distribution(random_engine);
			for (int j = 0; j < tao; j++){
				recurrent_transitions_vector[j].push_back(episodes_selected[i][starting_indice + j]);
			}
		}
		// Return recurrent_transitions_vector.
		return recurrent_transitions_vector;
	}

	int POMDP_ReplayMemory::size(){
		// Return the of the memory of the replay memory i.e. number of transition stored.
		return memory.size();
	}
}
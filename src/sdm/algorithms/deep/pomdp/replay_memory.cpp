#include <sdm/algorithms/deep/pomdp/replay_memory.hpp>

namespace sdm{
	POMDP_ReplayMemory::POMDP_ReplayMemory(int capacity, number tao, number eta, number horizon, number batch_size){
		this->capacity = capacity;
		this->tao = tao;
		this->eta = eta;
		this->horizon = horizon;
		this->batch_size = batch_size;
		this->uniform_transitions_distribution = std::uniform_int_distribution<int>(0, horizon - tao);
	}

	void POMDP_ReplayMemory::push(pomdp_transition t, int episode, number step){
		// Add the current transition to the
		current_episode_transitions.push_back(t);
		// If this transition belongs to the last step before the horizon
		if (step == horizon - 1){
			// Copy to transitions to a new object (to make sure there is no funny business with memory etc)
			pomdp_transitions_sequence new_episode_transitions = current_episode_transitions;
			// If the memory is not full
			if (memory.size() < capacity){
				// Add it to the end
				memory.push_back(new_episode_transitions);
			// If the memory is full
			} else {
				// Overwrite the oldest one in the memory
				memory[episode % capacity] = new_episode_transitions;
			}
			// Clear is so that the next episode can start fresh
			current_episode_transitions.clear();
		}
	}

	//
	std::vector<pomdp_transitions_sequence> POMDP_ReplayMemory::sample(){
		// Initialize the transitions squences to be returned.
		std::vector<pomdp_transitions_sequence> t_sequence(tao);
		// Episodes to be randomly selected from the experience replay memory.
		std::vector<pomdp_transitions_sequence> full_t_sequence;
		// Sample from the memory according to Mersenne Twister pseudorandom number generator and fill the full_t_sequence std::vector.
		std::experimental::sample(memory.begin(), memory.end(), std::back_inserter(full_t_sequence), batch_size / eta, std::mt19937{std::random_device{}()});

		//
		for (int i = 0; i < batch_size / eta; i++){
			//
			for (int e = 0; e < eta; e++){
				int starting_indice = uniform_transitions_distribution(random_engine);
				for (int t = 0; t < tao; t++){
					t_sequence[t].push_back(full_t_sequence[i][starting_indice + t]);
					// std::cout << "episode " << std::get<8>(full_t_sequence[i][starting_indice + t]) << std::endl;
					// std::cout << "step " << std::get<9>(full_t_sequence[i][starting_indice + t]) << std::endl << std::endl;
					// if (std::get<9>(full_t_sequence[i][starting_indice + t]) == 0){
					// 	std::cout << "episode " << std::get<8>(full_t_sequence[i][starting_indice + t]) << std::endl;
					// 	std::cout << "step " << std::get<9>(full_t_sequence[i][starting_indice + t]) << std::endl << std::endl;
					// }
				}
			}
		}
		// Return t_sequence.
		return t_sequence;
	}

	int POMDP_ReplayMemory::size(){
		// Return the of the memory of the replay memory i.e. number of transition stored.
		return memory.size();
	}
}
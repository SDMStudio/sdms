#include <sdm/algorithms/deep/osdpomdp/extensive_form_dql.hpp>

namespace sdm{
	ExtensiveFormDQL::ExtensiveFormDQL(
		int episodes, 
		number horizon, 
		number batch_size,
		number dim_o2,
		number dim_o1,
		number target_update, 
		number dim_i,
		number sampling_memory_size, 
		number print_every,
		number seed,
		float eps_end, 
		float eps_start, 
		float eps_decay, 
		float alpha_decay, 
		float rolling_factor, 
		float lr, 
		float adam_eps,
		torch::Device device,
		std::shared_ptr<sdm::POSG>& game,
		int replay_memory_size,
		bool induced_bias,
		std::string ib_net_filename
	){
		this->episodes = episodes;
		this->horizon = horizon;
		this->batch_size = batch_size;
		this->dim_o2 = dim_o2;
		this->dim_o1 = dim_o1;
		this->target_update = target_update;
		this->sampling_memory_size = sampling_memory_size;
		this->print_every = print_every;
		this->eps_end = eps_end;
		this->eps_start = eps_start;
		this->eps_decay = eps_decay;
		this->alpha_decay = alpha_decay;
		this->rolling_factor = rolling_factor;
		this->lr = lr;
		this->adam_eps = adam_eps;
		this->game = game;
		this->models_update_rules = std::make_shared<ModelsUpdateRules>(batch_size, sampling_memory_size, seed, device, game, induced_bias);
		this->agents =  std::make_shared<Agents>(
			game->getNumActions(0) + game->getNumObservations(0), dim_o2, 
			game->getNumActions(1) + game->getNumObservations(1), dim_o1,
			dim_o2 + dim_o1 + dim_o1 * sampling_memory_size + game->getNumStates(), dim_i, game->getNumActions(0) * game->getNumActions(1),
			seed,
			game, device, lr, adam_eps, induced_bias, ib_net_filename, sampling_memory_size
		);
		this->replay_memory = std::make_shared<ReplayMemory>(replay_memory_size, seed);
		this->GAMMA = game->getDiscount();
		this->uniform_m_distribution = std::uniform_int_distribution<int>(0, sampling_memory_size - 1); // not used, but will be later
		this->uniform_z2_distribution = std::uniform_int_distribution<int>(0, game->getNumObservations(0) - 1);
		this->induced_bias = induced_bias;
		initialize();
	}

	action ExtensiveFormDQL::get_a_from_u2_u1(action u2, action u1){
		std::vector<action> ja = {u2, u1};
		action a = game->getActionSpace().joint2single(ja);
		return a;
	}

	void ExtensiveFormDQL::estimate_initial_E_R(){
		// reward total_R = 0;
		// for (int episode = 0; episode < 1000; episode++){
		// 	reward R = 0;
		// 	state x = game->init();
		// 	for (int step = 0; step < horizon; step++){
		// 		action u2 = agent_0->uniform_action_distribution(agent_0->random_engine);
		// 		action u1 = agent_1->uniform_action_distribution(agent_1->random_engine);
		// 		action a = get_a_from_u2_u1(u2, u1);
		// 		std::tuple<std::vector<reward>, observation, state> r_z_x = game->getDynamicsGenerator(x, a);
		// 		reward r = std::get<0>(r_z_x)[0];
		// 		R += pow(game->getDiscount(), step) * r;
		// 		x = std::get<0>(r_z_x)[2];
		// 	}
		// 	total_R += R;
		// }
		// E_R = total_R / 1000;
		E_R = 0;
		// It was doing something weird so for now it'll stay like this, will revisit later.
	}

	void ExtensiveFormDQL::update_epsilon(){
		// If the training has not started yet, it will stay as 1.
		if (steps_done * sampling_memory_size < batch_size){
			epsilon = 1;
		// Otherwise it will start to decay.
		} else {
			epsilon = eps_end + (eps_start - eps_end) * exp(-1. * (steps_done * sampling_memory_size - batch_size) / eps_decay);
		}
	}

	void ExtensiveFormDQL::update_alpha(){
		// If the training has not started yet, it will stay as 1.
		if (steps_done * sampling_memory_size < batch_size){
			alpha = 1;
		// Otherwise it will start to decay.
		} else {
			alpha = exp(-1. * (steps_done * sampling_memory_size - batch_size) / alpha_decay);
		}
	}

	std::vector<state> ExtensiveFormDQL::initiate_xs(){
		std::vector<state> xs = {};
		state x = game->init();
		for(int j = 0; j < sampling_memory_size; j++){
			xs.push_back(x);
		}
		return xs;
	}

	std::vector<action> ExtensiveFormDQL::initiate_u1s(){
		std::vector<action> u1s = {};
		action u1 = 0;
		for(int j = 0; j < sampling_memory_size; j++){
			u1s.push_back(u1);
		}
		return u1s;
	}

	std::vector<observation> ExtensiveFormDQL::initiate_z1s(){
		std::vector<observation> z1s = {};
		observation z1 = 0;
		for(int j = 0; j < sampling_memory_size; j++){
			z1s.push_back(z1);
		}
		return z1s;
	}

	std::vector<history> ExtensiveFormDQL::initiate_o1s(){
		std::vector<history> o1s = {};
		history o1 = torch::zeros(dim_o1);
		for(int j = 0; j < sampling_memory_size; j++){
			o1s.push_back(o1);
		}
		return o1s;
	}

	std::vector<reward> ExtensiveFormDQL::initiate_rs(){
		std::vector<reward> rs = {};
		reward r = 0;
		for(int j = 0; j < sampling_memory_size; j++){
			rs.push_back(r);
		}
		return rs;
	}

	std::tuple<observation, observation, reward> ExtensiveFormDQL::act(){
		action u = get_a_from_u2_u1(u2, u1s[m]);
		std::tuple<std::vector<reward>, observation, state> r_z_next_x = game->getDynamicsGenerator(xs[m], u);
		std::vector<reward> rs = std::get<0>(r_z_next_x);
		observation z = std::get<1>(r_z_next_x);
		std::vector<observation> z2_z1 = game->getObsSpace().single2joint(z);
		next_xs[m] = std::get<2>(r_z_next_x);
		return std::make_tuple(z2_z1[0], z2_z1[1], rs[0]);
	}

	void ExtensiveFormDQL::initialize(){
		if (!induced_bias){
			std::cout << "Episode,Epsilon,Q Value Loss,E[R]" << std::endl;
		} else {
			std::cout << "Episode,Epsilon,Alpha,Q Value Loss,E[R]" << std::endl;
		}
		estimate_initial_E_R();
		update_epsilon();
		update_alpha();
		steps_done = 0;
	}

	void ExtensiveFormDQL::initialize_episode(){
		R = 0;
		q_value_loss = 0;
		xs = initiate_xs();
		next_xs = initiate_xs();
		o2 = torch::zeros(dim_o2);
		next_o2 = torch::zeros(dim_o2);
		o1s = initiate_o1s();
		next_o1s = initiate_o1s();
		u1s = initiate_u1s();
		z1s = initiate_z1s();
		rs = initiate_rs();
		p_x = create_state_probability_distribution(xs);
	}

	void ExtensiveFormDQL::update_replay_memory(){
		// First we have to create the next state probability distribution.
		p_next_x = create_state_probability_distribution(next_xs);
		// For each sampling/parallel world:
		for(m = 0; m < sampling_memory_size; m++){
			// Create transition (u2 + u1s[m] * game->getNumActions(0) - u. if you disagree let me know)
			transition t = std::make_tuple(o2, o1s[m], o1s, p_x, u2 + u1s[m] * game->getNumActions(0), rs[m], next_o2, next_o1s[m], next_o1s, p_next_x);
			// Push it to the replay memory.
			replay_memory->push(t);
		}
	}

	void ExtensiveFormDQL::end_step(){
		// Update the state.
		xs = next_xs;
		// Update the history of agent 2.
		o2 = next_o2;
		// Update the histories of agent 1.
		o1s = next_o1s;
		// Update the state probability distribution.
		p_x = p_next_x;
		// Update epsilon, the exploration coefficient.
		update_epsilon();
		// Update alpha, the coefficient for how much of the IB target we use during the updates.
		update_alpha();
		// Increment steps done.
		steps_done++;
	}

	void ExtensiveFormDQL::end_episode(){
		// Apply rolling to it to get smoother values.
		E_R = E_R * rolling_factor + R * (1 - rolling_factor);
		// If we want to print:
		if(episode % print_every == 0){
			if(!induced_bias){
				std::cout << episode << "," << epsilon << "," << q_value_loss / horizon << "," << E_R << std::endl;
			} else {
				std::cout << episode << "," << epsilon << "," << alpha << ","  << q_value_loss / horizon << "," << E_R << std::endl;
			}
		}
		// If we want to update the target network:
		if(episode % target_update == 0){
			// Update the target net using policy net.
			agents->update_target_net();
		}
	}

	void ExtensiveFormDQL::update_models(){
		// Update weights and get q loss.
		q_value_loss += models_update_rules->update(replay_memory, agents, alpha);
	}

	state_probability_distribution ExtensiveFormDQL::create_state_probability_distribution(std::vector<state> xs){
		state_probability_distribution p_x = torch::zeros(game->getNumStates());
		for(m = 0; m < sampling_memory_size; m++){
			p_x.index({xs[m]}) += 1.0 / sampling_memory_size;
		}
		return p_x;
	}

	void ExtensiveFormDQL::solve(){
		// For each episode, until we reach the total number episodes:
		for(episode = 0; episode < episodes; episode++){
			// Initialize all the objects that will be used during the episode.
			initialize_episode();
			// For each step, until we reach the horizon:
			for(step = 0; step < horizon; step++){
				// Agent 2 chooses her action u2.
				u2 = agents->get_epsilon_greedy_action_2(o2, o1s, p_x, epsilon);
				// We randomly choose a Agent 2' observation z2.
				z2 = uniform_z2_distribution(random_engine);
				// For each sampling/parallel world until we reach the sampling_memory_size:
				for(m = 0; m < sampling_memory_size; m++){
					// Agent 1 chooses her action u1s[m].
					u1s[m] = agents->get_epsilon_greedy_action_1(o2, o1s[m], u2, o1s, p_x, epsilon);
					do {
						// Agents act according to their chosen actions, we receive a candidate observation for agent 2 candidate_z2, observation for agent 1 z1s[m], and reward rs[m].
						std::tie(candidate_z2, z1s[m], rs[m]) = act();
					// Candidate z2 must be the same as z2, it's the only eay to make sure that z1s[m] and rs[m] are possible given z2.
					} while (candidate_z2 != z2);
					// Agent 1's history is updated using to her private history, private action, and private observation.
					next_o1s[m] = agents->get_next_history_1(o1s[m], u1s[m], z1s[m]);
					// The discounted cumulative reward is updated according to GAMMA, step, rs[m], and sampling_memory_size.
					R += pow(GAMMA, step) * (rs[m] / sampling_memory_size);	
				}
				// Agent 2's history is updated using to her private history, private action, and private observation.
				next_o2 = agents->get_next_history_2(o2, u2, z2);
				// We update the replay memory with the transition that just occurred.
				update_replay_memory();
				// We update models using the data in replay memory.
				update_models();
				// Do what is needed to be ready for the next state.
				end_step();
			}
			// Do what is neccessary to finish the episode.
			end_episode();
		}
	}
}
#include <sdm/algorithms/deep/osdpomdp/extensive_form_dql.hpp>

namespace sdm{
	ExtensiveFormDQL::ExtensiveFormDQL(
		int episodes, 
		number horizon, 
		number batch_size,
		number n,
		number target_update, 
		number dim_i1,
		number sampling_memory_size, 
		number print_every,
		float eps_end, 
		float eps_start, 
		float eps_decay, 
		float alpha_decay, 
		float discount_factor, 
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
		this->n = n;
		this->target_update = target_update;
		this->sampling_memory_size = sampling_memory_size;
		this->print_every = print_every;
		this->eps_end = eps_end;
		this->eps_start = eps_start;
		this->eps_decay = eps_decay;
		this->alpha_decay = alpha_decay;
		this->discount_factor = discount_factor;
		this->rolling_factor = rolling_factor;
		this->lr = lr;
		this->adam_eps = adam_eps;
		this->game = game;
		this->models_update_rules = std::make_shared<ModelsUpdateRules>(batch_size, sampling_memory_size, device, game, induced_bias);
		this->agents =  std::make_shared<Agents>(
			(game->getNumActions(0) + game->getNumObservations(0)) * n + (game->getNumActions(1) + game->getNumObservations(1)) * n + game->getNumActions(0) + (game->getNumActions(1) + game->getNumObservations(1)) * n * sampling_memory_size, 
			dim_i1, game->getNumActions(1),
			game, device, lr, adam_eps, induced_bias, ib_net_filename, sampling_memory_size
		);
		this->replay_memory = std::make_shared<ReplayMemory>(replay_memory_size);
		this->GAMMA = game->getDiscount();
		this->uniform_m_distribution = std::uniform_int_distribution<int>(0, sampling_memory_size - 1);
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
	}

	void ExtensiveFormDQL::update_epsilon(){
		if (steps_done * sampling_memory_size < batch_size){
			epsilon = 1;
		} else {
			epsilon = eps_end + (eps_start - eps_end) * exp(-1. * (steps_done * sampling_memory_size - batch_size) / eps_decay);
		}
	}

	void ExtensiveFormDQL::update_alpha(){
		if (steps_done * sampling_memory_size < batch_size){
			alpha = 1;
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
		history o1 = torch::zeros((game->getNumActions(1) + game->getNumObservations(1)) * n);
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
		o2 = torch::zeros((game->getNumActions(0) + game->getNumObservations(0)) * n);
		next_o2 = torch::zeros((game->getNumActions(0) + game->getNumObservations(0)) * n);
		o1s = initiate_o1s();
		next_o1s = initiate_o1s();
		u1s = initiate_u1s();
		z1s = initiate_z1s();
		rs = initiate_rs();
		m_star = 0;
	}

	void ExtensiveFormDQL::update_replay_memory(){
		for(m = 0; m < sampling_memory_size; m++){
			// Create transition
			transition t = std::make_tuple(o2, o1s[m], o1s, u2, u1s[m], rs[m], next_o2, next_o1s[m], next_o1s);
			// Push it to the replay memory.
			replay_memory->push(t);
		}
	}

	void ExtensiveFormDQL::determine_branch(){
		m_star = uniform_m_distribution(random_engine);
		// m_star = sampling_memory_size - 1;
	}

	void ExtensiveFormDQL::end_step(){
		//
		determine_branch();
		// Update the state.
		xs = next_xs;
		// Update the history of agent 2.
		o2 = next_o2;
		// Update the histories of agent 1.
		o1s = next_o1s;
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
		//
		if(episode % print_every == 0){
			if(!induced_bias){
				std::cout << episode << "," << epsilon << "," << q_value_loss << "," << E_R << std::endl;
			} else {
				std::cout << episode << "," << epsilon << "," << alpha << ","  << q_value_loss << "," << E_R << std::endl;
			}
		}
		if(episode % target_update == 0){
			// Update the target net using policy net of agent 1.
			agents->update_target_net();
		}
	}

	void ExtensiveFormDQL::update_models(){
		// Update weights and get q loss.
		q_value_loss = models_update_rules->update(replay_memory, agents, alpha);
	}

	void ExtensiveFormDQL::solve(){
		for(episode = 0; episode < episodes; episode++){
			initialize_episode();
			for(step = 0; step < horizon; step++){
				u2 = agents->get_epsilon_greedy_action_2(o2, o1s, epsilon);
				z2 = uniform_z2_distribution(random_engine);
				for(m = 0; m < sampling_memory_size; m++){
					u1s[m] = agents->get_epsilon_greedy_action_1(o2, o1s[m], u2, o1s, epsilon);
					do {
						std::tie(candidate_z2, z1s[m], rs[m]) = act();
					} while (candidate_z2 != z2);
					next_o1s[m] = agents->get_next_history_1(o1s[m], u1s[m], z1s[m]);
					R += pow(GAMMA, step) * (rs[m] / sampling_memory_size);	
				}
				next_o2 = agents->get_next_history_2(o2, u2, z2);
				update_replay_memory();
				update_models();
				end_step();
			}
			end_episode();
		}
	}
}
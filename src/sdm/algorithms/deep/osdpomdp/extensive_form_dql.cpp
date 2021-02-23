#include <sdm/algorithms/deep/osdpomdp/extensive_form_dql.hpp>

namespace sdm{
	ExtensiveFormDQL::ExtensiveFormDQL(
		int episodes, 
		number horizon, 
		number batch_size,
		number i_batch_size, 
		number dim_o2, 
		number dim_o1, 
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
		std::string output_file_name,
		bool induced_bias,
		std::string ib_net_filename
	){
		this->episodes = episodes;
		this->horizon = horizon;
		this->batch_size = batch_size;
		this->i_batch_size = i_batch_size;
		this->dim_o2 = dim_o2;
		this->dim_o1 = dim_o1;
		this->target_update = target_update;
		this->dim_i1 = dim_i1;
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
			game->getNumActions(0) + game->getNumObservations(0), dim_o2, 
			game->getNumActions(1) + game->getNumObservations(1), dim_o1,
			dim_o2 + dim_o1 + game->getNumActions(0) + dim_o1 * sampling_memory_size, dim_i1, game->getNumActions(1),
			game, device, lr, adam_eps, induced_bias, ib_net_filename, sampling_memory_size
		);
		this->replay_memory = std::make_shared<ReplayMemory>(replay_memory_size);
		this->output_file.open(output_file_name);
		this->GAMMA = game->getDiscount();
		this->uniform_m_distribution = std::uniform_int_distribution<int>(0, sampling_memory_size - 1);
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
		if (steps_done < batch_size){
			epsilon = 1;
		} else {
			epsilon = eps_end + (eps_start - eps_end) * exp(-1. * steps_done / eps_decay);
		}
	}

	void ExtensiveFormDQL::update_alpha(){
		if (steps_done < batch_size){
			alpha = 1;
		} else {
			alpha = exp(-1. * steps_done / alpha_decay);
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
		action u = get_a_from_u2_u1(u2[i], u1s[i][m]);
		std::tuple<std::vector<reward>, observation, state> r_z_next_x = game->getDynamicsGenerator(xs[i][m_star], u);
		std::vector<reward> rs = std::get<0>(r_z_next_x);
		observation z = std::get<1>(r_z_next_x);
		std::vector<observation> z2_z1 = game->getObsSpace().single2joint(z);
		next_xs[i][m] = std::get<2>(r_z_next_x);
		return std::make_tuple(z2_z1[0], z2_z1[1], rs[0]);
	}

	void ExtensiveFormDQL::initialize(){
		if (!induced_bias){
			output_file << "Episode,Epsilon,Q Value Loss,E[R]" << std::endl;
			std::cout << "Episode,Epsilon,Q Value Loss,E[R]" << std::endl;
		} else {
			output_file << "Episode,Epsilon,Alpha,Q Value Loss,E[R]" << std::endl;
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
		xs = {};
		for(int i = 0; i < i_batch_size; i++){
			xs.push_back(initiate_xs());
		}
		next_xs = {};
		for(int i = 0; i < i_batch_size; i++){
			next_xs.push_back(initiate_xs());
		}
		o2s = {};
		for(int i = 0; i < i_batch_size; i++){
			o2s.push_back(initiate_o1s());
		}
		next_o2s = {};
		for(int i = 0; i < i_batch_size; i++){
			next_o2s.push_back(initiate_o1s());
		}
		o1s = {};
		for(int i = 0; i < i_batch_size; i++){
			o1s.push_back(initiate_o1s());
		}
		next_o1s = {};
		for(int i = 0; i < i_batch_size; i++){
			next_o1s.push_back(initiate_o1s());
		}
		u2 = {};
		for(int i = 0; i < i_batch_size; i++){
			u2.push_back(0);
		}
		u1s = {};
		for(int i = 0; i < i_batch_size; i++){
			u1s.push_back(initiate_u1s());
		}
		z2s = {};
		for(int i = 0; i < i_batch_size; i++){
			z2s.push_back(initiate_z1s());
		}
		z1s = {};
		for(int i = 0; i < i_batch_size; i++){
			z1s.push_back(initiate_z1s());
		}
		rs = {};
		for(int i = 0; i < i_batch_size; i++){
			rs.push_back(initiate_rs());
		}
		m_star = 0;
	}

	void ExtensiveFormDQL::update_replay_memory(){
		for(m = 0; m < sampling_memory_size; m++){
			// Create transition
			transition t = std::make_tuple(o2s[i][m_star], o1s[i][m], o1s[i], u2[i], u1s[i][m], rs[i][m], next_o2s[i][m], next_o1s[i][m], next_o1s[i]);
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
		xs[i] = next_xs[i];
		// Update the histories of agent 2.
		o2s[i] = next_o2s[i];
		// Update the histories of agent 1.
		o1s[i] = next_o1s[i];
		// Update epsilon, the exploration coefficient.
		update_epsilon();
		// Update alpha, the coefficient for how much of the IB target we use during the updates.
		update_alpha();
		// If number of steps is a multiple of target_update hyperparameter:
		if((episode * horizon + step) % target_update == 0){
			// Update the target net using policy net of agent 1.
			agents->update_target_net();
		}
		// Increment steps done.
		steps_done++;
	}

	void ExtensiveFormDQL::end_episode(){
		// Apply rolling to it to get smoother values.
		E_R = E_R * rolling_factor + R * (1 - rolling_factor);
		//
		if(episode % print_every == 0){
			if(!induced_bias){
				output_file << episode << "," << epsilon << "," << q_value_loss << "," << E_R << std::endl;
				std::cout << episode << "," << epsilon << "," << q_value_loss << "," << E_R << std::endl;
			} else {
				output_file << episode << "," << epsilon << "," << alpha << ","  << q_value_loss << "," << E_R << std::endl;
				std::cout << episode << "," << epsilon << "," << alpha << ","  << q_value_loss << "," << E_R << std::endl;
			}
			
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
				for(i = 0; i < i_batch_size; i++){
					u2[i] = agents->get_epsilon_greedy_action_2(o2s[i][m_star], o1s[i], epsilon);
					for(m = 0; m < sampling_memory_size; m++){
						u1s[i][m] = agents->get_epsilon_greedy_action_1(o2s[i][m_star], o1s[i][m_star], u2[i], o1s[i], epsilon);
						std::tie(z2s[i][m], z1s[i][m], rs[i][m]) = act();
						next_o2s[i][m] = agents->get_next_history_2(o2s[i][m_star], u2[i], z2s[i][m]);
						next_o1s[i][m] = agents->get_next_history_1(o1s[i][m_star], u1s[i][m], z1s[i][m]);
						R += pow(GAMMA, step) * (rs[i][m] / (i_batch_size * sampling_memory_size));	
					}
					update_replay_memory();
					update_models();
					end_step();
				}
			}
			end_episode();
		}
	}
}
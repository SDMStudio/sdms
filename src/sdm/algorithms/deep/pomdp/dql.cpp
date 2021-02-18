#include <sdm/algorithms/deep/pomdp/dql.hpp>

namespace sdm{
	DQL::DQL(
		int episodes, 
		number horizon, 
		number batch_size,
		number i_batch_size, 
		number dim_o2, 
		number dim_o1, 
		number target_update, 
		number dim_i,
		number print_every,
		float eps_end, 
		float eps_start, 
		float eps_decay,  
		float discount_factor, 
		float rolling_factor, 
		float lr, 
		float adam_eps,
		torch::Device device,
		std::shared_ptr<sdm::POSG>& game,
		int replay_memory_size,
		std::string output_file_name,
		std::string ib_net_filename
	){
		std::cout << "hey" << std::endl;
		this->episodes = episodes;
		this->horizon = horizon;
		this->batch_size = batch_size;
		this->i_batch_size = i_batch_size;
		this->dim_o2 = dim_o2;
		this->dim_o1 = dim_o1;
		this->target_update = target_update;
		this->dim_i = dim_i;
		this->print_every = print_every;
		this->eps_end = eps_end;
		this->eps_start = eps_start;
		this->eps_decay = eps_decay;
		this->discount_factor = discount_factor;
		this->rolling_factor = rolling_factor;
		this->lr = lr;
		this->adam_eps = adam_eps;
		this->game = game;
		std::cout << "a" << std::endl;
		this->models_update_rules = std::make_shared<POMDP_ModelsUpdateRules>(batch_size, device, game);
		std::cout << "b" << std::endl;
		this->agents =  std::make_shared<POMDP_Agents>(
			game->getNumActions(0) + game->getNumObservations(0), dim_o2, 
			game->getNumActions(1) + game->getNumObservations(1), dim_o1,
			dim_o2 + dim_o1, dim_i, game->getNumActions(0) * game->getNumActions(1),
			game, device, lr, adam_eps, ib_net_filename
		);
		std::cout << "c" << std::endl;
		this->replay_memory = std::make_shared<POMDP_ReplayMemory>(replay_memory_size);
		std::cout << "d" << std::endl;
		this->output_file.open(output_file_name);
		this->GAMMA = game->getDiscount();
		std::cout << "yo" << std::endl;
		initialize();
		std::cout << "man" << std::endl;
	}

	action DQL::get_a_from_u2_u1(action u2, action u1){
		std::vector<action> ja = {u2, u1};
		action a = game->getActionSpace().joint2single(ja);
		return a;
	}

	void DQL::estimate_initial_E_R(){
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

	void DQL::update_epsilon(){
		if (steps_done < batch_size){
			epsilon = 1;
		} else {
			epsilon = eps_end + (eps_start - eps_end) * exp(-1. * steps_done / eps_decay);
		}
	}

	std::tuple<observation, observation, reward> DQL::act(){
		action u = get_a_from_u2_u1(u2[i], u1[i]);
		std::tuple<std::vector<reward>, observation, state> r_z_next_x = game->getDynamicsGenerator(x[i], u);
		std::vector<reward> rs = std::get<0>(r_z_next_x);
		observation z = std::get<1>(r_z_next_x);
		std::vector<observation> z2_z1 = game->getObsSpace().single2joint(z);
		next_x[i] = std::get<2>(r_z_next_x);
		return std::make_tuple(z2_z1[0], z2_z1[1], rs[0]);
	}

	void DQL::initialize(){

		output_file << "Episode,Epsilon,Q Value Loss,E[R]" << std::endl;
		std::cout << "Episode,Epsilon,Q Value Loss,E[R]" << std::endl;

		estimate_initial_E_R();
		update_epsilon();
		steps_done = 0;
	}

	void DQL::initialize_episode(){
		R = 0;
		q_value_loss = 0;
		x = {};
		for(int i = 0; i < i_batch_size; i++){
			x.push_back(game->init());
		}
		next_x = {};
		for(int i = 0; i < i_batch_size; i++){
			next_x.push_back(game->init());
		}
		o2 = {};
		for(int i = 0; i < i_batch_size; i++){
			o2.push_back(torch::zeros(dim_o2));
		}
		next_o2 = {};
		for(int i = 0; i < i_batch_size; i++){
			next_o2.push_back(torch::zeros(dim_o2));
		}
		o1 = {};
		for(int i = 0; i < i_batch_size; i++){
			o1.push_back(torch::zeros(dim_o1));
		}
		next_o1 = {};
		for(int i = 0; i < i_batch_size; i++){
			next_o1.push_back(torch::zeros(dim_o1));
		}
		u2 = {};
		for(int i = 0; i < i_batch_size; i++){
			u2.push_back(0);
		}
		u1 = {};
		for(int i = 0; i < i_batch_size; i++){
			u1.push_back(0);
		}
		u2_u1 = {};
		for(int i = 0; i < i_batch_size; i++){
			u2_u1.push_back(0);
		}
		z1 = {};
		for(int i = 0; i < i_batch_size; i++){
			z1.push_back(0);
		}
		z2 = {};
		for(int i = 0; i < i_batch_size; i++){
			z2.push_back(0);
		}
		r = {};
		for(int i = 0; i < i_batch_size; i++){
			r.push_back(0);
		}
	}

	void DQL::update_replay_memory(){
		// Create transition
		pomdp_transition t = std::make_tuple(o2[i], o1[i], u2[i], u1[i], u2_u1[i], z2[i], z1[i], r[i]);
		// Push it to the replay memory.
		replay_memory->push(t);
	}


	void DQL::end_step(){
		// Update the state.
		x[i] = next_x[i];
		// Update the history of agent 2.
		o2[i] = next_o2[i];
		// Update the history of agent 1.
		o1[i] = next_o1[i];
		// Update epsilon, the exploration coefficient.
		update_epsilon();
		// If number of steps is a multiple of target_update hyperparameter:
		if((episode * horizon + step) % target_update == 0){
			// Update the target net using policy net of agent 1.
			agents->update_target_net();
		}
		// Increment steps done.
		steps_done++;
	}

	void DQL::end_episode(){
		// Apply rolling to it to get smoother values.
		E_R = E_R * rolling_factor + R * (1 - rolling_factor);
		//
		if(episode % print_every == 0){
			output_file << episode << "," << epsilon << "," << q_value_loss << "," << E_R << std::endl;
			std::cout << episode << "," << epsilon << "," << q_value_loss << "," << E_R << std::endl;
		}
	}

	void DQL::update_models(){
		// Update weights and get q loss.
		q_value_loss = models_update_rules->update(replay_memory, agents);
	}

	void DQL::solve(){
		for(episode = 0; episode < episodes; episode++){
			initialize_episode();
			for(step = 0; step < horizon; step++){
				for(i = 0; i < i_batch_size; i++){
					u2_u1[i] = agents->get_epsilon_greedy_actions(o2[i], o1[i], epsilon);
					u2[i] = u2_u1[i] % game->getNumActions(0);
					u1[i] = u2_u1[i] / game->getNumActions(0);
					std::tie(z2[i], z1[i], r[i]) = act();
					update_replay_memory();
					update_models();
					next_o2[i] = agents->get_next_history_2(o2[i], u2[i], z2[i]);
					next_o1[i] = agents->get_next_history_1(o1[i], u1[i], z1[i]);
					R += pow(GAMMA, step) * (r[i] / i_batch_size);
					end_step();
				}
			}
			end_episode();
		}
	}
}
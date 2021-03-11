#include <sdm/algorithms/deep/pomdp_simple/dql.hpp>

namespace sdm{
	DQL_Simple::DQL_Simple(
		int episodes, 
		number horizon, 
		number batch_size,
		number n,  
		number dim_i,
		number target_update, 
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
		std::string ib_net_filename
	){
		this->episodes = episodes;
		this->horizon = horizon;
		this->batch_size = batch_size;
		this->n = n;
		this->target_update = target_update;
		this->print_every = print_every;
		this->eps_end = eps_end;
		this->eps_start = eps_start;
		this->eps_decay = eps_decay;
		this->discount_factor = discount_factor;
		this->rolling_factor = rolling_factor;
		this->lr = lr;
		this->adam_eps = adam_eps;
		this->game = game;
		this->models_update_rules = std::make_shared<POMDP_ModelsUpdateRules_Simple>(batch_size, device, game);
		this->agents =  std::make_shared<POMDP_Agents_Simple>(
			(game->getNumActions(0) + game->getNumObservations(0)) * n + (game->getNumActions(1) + game->getNumObservations(1)) * n, 
			dim_i, game->getNumActions(0) * game->getNumActions(1),
			game, device, lr, adam_eps, ib_net_filename
		);
		this->replay_memory = std::make_shared<POMDP_ReplayMemory_Simple>(replay_memory_size, batch_size);
		this->GAMMA = game->getDiscount();
		initialize();
	}

	action DQL_Simple::get_u_from_u2_u1(action u2, action u1){
		std::vector<action> ja = {u2, u1};
		action u = game->getActionSpace().joint2single(ja);
		return u;
	}

	void DQL_Simple::update_epsilon(){
		if (steps_done < batch_size){
			epsilon = 1;
		} else {
			epsilon = eps_end + (eps_start - eps_end) * exp(-1. * (steps_done - batch_size) / eps_decay);
			if (epsilon < .001){
				epsilon = 0;
			}
		}
	}

	std::tuple<observation, observation, reward> DQL_Simple::act(){
		action u = get_u_from_u2_u1(u2, u1);
		std::tuple<std::vector<reward>, observation, state> r_z_next_x = game->getDynamicsGenerator(x, u);
		std::vector<reward> rs = std::get<0>(r_z_next_x);
		observation z = std::get<1>(r_z_next_x);
		std::vector<observation> z2_z1 = game->getObsSpace().single2joint(z);
		next_x = std::get<2>(r_z_next_x);
		return std::make_tuple(z2_z1[0], z2_z1[1], rs[0]);
	}

	void DQL_Simple::initialize(){
		std::cout << "Episode,Epsilon,Q Value Loss,E[R]" << std::endl;
		update_epsilon();
		steps_done = 0;
		E_R = 0;
	}

	void DQL_Simple::initialize_episode(){
		R = 0;
		q_value_loss = 0;
		x = game->init();
		o2 = torch::zeros((game->getNumActions(0) + game->getNumObservations(0)) * n);
		o1 = torch::zeros((game->getNumActions(1) + game->getNumObservations(1)) * n);
	}

	void DQL_Simple::update_replay_memory(){
		// Create transition
		pomdp_transition_simple t = std::make_tuple(o2, o1, u2_u1, r, next_o2, next_o1); // ad next histories
		// Push it to the replay memory.
		replay_memory->push(t);
	}


	void DQL_Simple::end_step(){
		// Update the state.
		x = next_x;
		// Update the history of agent 2.
		o2 = next_o2;
		// Update the history of agent 1.
		o1 = next_o1;
		// Update epsilon, the exploration coefficient.
		update_epsilon();
		// Increment steps done.
		steps_done++;
	}

	void DQL_Simple::end_episode(){
		// Apply rolling to it to get smoother values.
		E_R = E_R * rolling_factor + R * (1 - rolling_factor);
		//
		if(episode % print_every == 0){
			std::cout << episode << "," << epsilon << "," << q_value_loss / horizon << "," << E_R << std::endl;
		}
		if(episode % target_update == 0){
			// Update the target nets.
			agents->update_target_nets();
		}
	}

	void DQL_Simple::update_models(){
		// Update weights and get q loss.
		q_value_loss += models_update_rules->update(replay_memory, agents);
	}

	void DQL_Simple::solve(){
		for(episode = 0; episode < episodes; episode++){
			initialize_episode();
			for(step = 0; step < horizon; step++){
				u2_u1 = agents->get_epsilon_greedy_actions(o2, o1, epsilon);
				u2 = u2_u1 % game->getNumActions(0);
				u1 = u2_u1 / game->getNumActions(0);
				std::tie(z2, z1, r) = act();
				update_replay_memory();
				update_models();
				next_o2 = agents->get_next_history_2(o2, u2, z2);
				next_o1 = agents->get_next_history_1(o1, u1, z1);
				R += pow(GAMMA, step) * r;
				end_step();
			}
			end_episode();
		}
	}
}
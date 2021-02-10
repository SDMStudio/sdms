#include <sdm/world/gym_interface.hpp>

namespace sdm
{

    template <typename TState, typename TAction, typename TObservation>
    LearningAlgo<TState, TAction, TObservation>::LearningAlgo(number num_max_trials, std::string results) : max_trials(num_max_trials)
    {
    }

    template <typename TState, typename TAction, typename TObservation>
    LearningAlgo<TState, TAction, TObservation>::LearningAlgo(std::shared_ptr<GymInterface> env,
                                                              number planning_horizon,
                                                              number num_max_trials = 10000) : max_trials(num_max_trials), planning_horizon_(planning_horizon), env_(env)
    {
    }

    template <typename TState, typename TAction, typename TObservation>
    void LearningAlgo<TState, TAction, TObservation>::do_initialize()
    {
        // assert timestep_init >=0, "Initial timestep must be upper or equal than 0"
        // assert timestep_init < nb_timesteps, "Initial timestep must be lower than the total number of timesteps"

        // start_time = datetime.now()
        // logging.basicConfig()

        // reset_logging()
        // if log_file is None:
        //     logging.basicConfig(stream=sys.stdout, format='%(message)s', level=logging.INFO)
        // else:
        //     logging.basicConfig(filename=os.path.join(self.log_dir,log_file), format='%(message)s', level=logging.INFO)
    }

    template <typename TState, typename TAction, typename TObservation>
    void LearningAlgo<TState, TAction, TObservation>::do_solve(std::shared_ptr<GymInterface> env, long nb_timesteps, number planning_horizon, number test_freq, number save_freq, std::string save_folder, number verbose, long timestep_init, std::string log_file)
    {
        std::cout << "#> Start learning process.\n|\tDate : " << std::endl;
        this->trial = timestep_init;
        this->planning_horizon_ = planning_horizon;
        this->env_ = env;

        double best_rew = -10000; // self.worst_rew();
        bool test = false, done = false;
        this->exploration_process->reset_exploration(nb_timesteps);
        
        while (this->trial < nb_timesteps)
        {
            this->exploration_process->update_exploration(this->trial);
            episode += 1;
            TObservation obs = env->reset();

            long ctrial = this->trial, stop_cond = this->trial + planning_horizon;
            for (ctrial; ctrial < stop_cond; ctrial++)
            {
                // Action selection following policy and exploration process
                auto action = this->select_action(obs);

                // One step in env and get next observation and rewards
                TObservation next_obs;
                std::vector<double> rewards;
                std::tie(next_obs, rewards, done) = env->step(action);

                // Store experience in the associated buffer
                this->store_experience(obs, action, rewards, next_obs, done);
                obs = next_obs;

                // Update the model
                this->update_model(this->trial);
                this->trial++;

                //Save the model
                if (this->trial % save_freq == 0)
                {
                    logging.info("#> Step {}/{} --- Save Model\n".format(this->trial, nb_timesteps));
                    this->save_policy(this->trial, save_folder);
                }
                // Test the model
                if (this->trial % test_freq == 0)
                {
                    test = true;
                }
                if (done)
                {
                    break;
                }
            }
        }
    }
} // namespace sdm
#include <sdm/algorithms/q_learning.hpp>

namespace sdm
{

    template <typename TState, typename TAction, typename TObservation>
    QLearning<TState, TAction, TObservation>::QLearning(std::shared_ptr<GymInterface> env,
                                                        std::shared_ptr<QValueFunction<TState, TAction>> q_value,
                                                        std::shared_ptr<QValueFunction<TState, TAction>> q_target,
                                                        number planning_horizon,
                                                        double discount,
                                                        double lr,
                                                        double batch_size,
                                                        number num_max_steps,
                                                        std::string name) : env_(env),
                                                                            q_value_(q_value),
                                                                            q_target_(q_target),
                                                                            planning_horizon_(planning_horizon),
                                                                            discount_(discount),
                                                                            lr_(lr),
                                                                            batch_size_(batch_size),
                                                                            max_steps_(num_max_steps),
                                                                            name_(name)
    {
    }

    template <typename TState, typename TAction, typename TObservation>
    void QLearning<TState, TAction, TObservation>::initLogger()
    {
        std::string format = "#> Trial : {}\tError : {}\t\tV_lb({}) / V_ub({})\n";

        auto std_logger = std::make_shared<sdm::StdLogger>(format);
        auto file_logger = std::make_shared<sdm::FileLogger>(this->name_ + ".txt", format);
        auto csv_logger = std::make_shared<sdm::CSVLogger>(this->name_, std::vector<std::string>{"Trial", "Error", "Value_LB", "Value_UB", "Time"});

        this->logger_ = std::make_shared<sdm::MultiLogger>(std::vector<std::shared_ptr<Logger>>{std_logger, file_logger, csv_logger});
    }

    template <typename TState, typename TAction, typename TObservation>
    void QLearning<TState, TAction, TObservation>::do_initialize()
    {
        this->initLogger();

        this->q_value_->initialize();
        this->q_target_->initialize();
    }

    // std::shared_ptr<GymInterface> env, long nb_timesteps, number planning_horizon, number test_freq, number save_freq, std::string save_folder, number verbose, long timestep_init, std::string log_file
    template <typename TState, typename TAction, typename TObservation>
    void QLearning<TState, TAction, TObservation>::do_solve()
    {
        this->step = 0;

        clock_t t_begin = clock();

        this->exploration_process->reset_exploration(this->max_steps_);

        while (this->step < this->max_steps_)
        {
            // Update exploration process
            this->exploration_process->update_exploration(this->step);

            // Do one episode
            this->do_episode();

            // Test current policy and write logs
            if (this->do_log)
            {
                this->logger_->log(this->step,
                                   (float)(clock() - t_begin) / CLOCKS_PER_SEC);
                this->do_log = false;
            }
            if (this->do_test)
            {
                std::cout << "--------- TESTING ---------" << std::endl;
                this->do_test();
                this->do_test = false;
            }
        }
    }

    template <typename TState, typename TAction, typename TObservation>
    void QLearning<TState, TAction, TObservation>::do_test()
    {
    }

    template <typename TState, typename TAction, typename TObservation>
    void QLearning<TState, TAction, TObservation>::do_episode()
    {
        this->episode += 1;
        this->current_obs = env->reset();

        long stop_cond = this->step + this->planning_horizon_;
        while (this->step < stop_cond)
        {
            this->do_step();

            //Save the model
            this->do_save = (this->step % this->save_freq == 0);
            this->do_log = (this->step % this->log_freq == 0);
            this->do_test = (this->step % this->test_freq == 0);

            if (this->is_done)
            {
                break;
            }
        }
    }

    template <typename TState, typename TAction, typename TObservation>
    void QLearning<TState, TAction, TObservation>::do_step()
    {
        // Action selection following policy and exploration process
        auto action = this->select_action(this->current_obs);

        // One step in env and get next observation and rewards
        auto [next_obs, rewards, done] = env->step(action);

        // Store experience in the associated buffer
        this->experience->store_experience(this->current_obs, action, rewards, next_obs, done);

        this->current_obs = next_obs;
        this->is_done = done;

        // Update the model
        this->q_value_->updateQValueAt(s, h);

        this->step++;
    }

} // namespace sdm

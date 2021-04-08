#include <sdm/algorithms/q_learning.hpp>

namespace sdm
{

    template <typename TObservation, typename TAction>
    QLearning<TObservation, TAction>::QLearning(std::shared_ptr<GymInterface<TObservation, TAction>> &env,
                                                std::shared_ptr<QValueFunction<TObservation, TAction>> q_value,
                                                std::shared_ptr<QValueFunction<TObservation, TAction>> q_target,
                                                std::shared_ptr<EpsGreedy<TObservation, TAction>> exploration,
                                                number planning_horizon,
                                                double discount,
                                                double lr,
                                                double batch_size,
                                                number num_max_steps,
                                                std::string name) : env_(env),
                                                                    q_value_(q_value),
                                                                    q_target_(q_target),
                                                                    exploration_process(exploration),
                                                                    planning_horizon_(planning_horizon),
                                                                    discount_(discount),
                                                                    lr_(lr),
                                                                    batch_size_(batch_size),
                                                                    max_steps_(num_max_steps),
                                                                    name_(name)
    {
    }

    template <typename TObservation, typename TAction>
    void QLearning<TObservation, TAction>::initLogger()
    {
        std::string format = "#> Episode : {}\tStep : {}/?\tQValue : {}\n";

        auto std_logger = std::make_shared<sdm::StdLogger>(format);
        auto file_logger = std::make_shared<sdm::FileLogger>(this->name_ + ".txt", format);
        auto csv_logger = std::make_shared<sdm::CSVLogger>(this->name_, std::vector<std::string>{"Episode", "Step", "QValue", "Time"});

        this->logger_ = std::make_shared<sdm::MultiLogger>(std::vector<std::shared_ptr<Logger>>{std_logger, file_logger, csv_logger});
    }

    template <typename TObservation, typename TAction>
    void QLearning<TObservation, TAction>::do_initialize()
    {
        this->initLogger();

        this->q_value_->initialize();
        this->q_target_->initialize();
    }

    // std::shared_ptr<GymInterface> env, long nb_timesteps, number planning_horizon, number test_freq, number save_freq, std::string save_folder, number verbose, long timestep_init, std::string log_file
    template <typename TObservation, typename TAction>
    void QLearning<TObservation, TAction>::do_solve()
    {
        this->global_step = 0;

        clock_t t_begin = clock();

        this->exploration_process->reset(this->max_steps_);

        while (this->global_step < this->max_steps_)
        {
            // Update exploration process
            this->exploration_process->update(this->global_step);

            // Do one episode
            this->do_episode();

            // Test current policy and write logs
            if (this->do_log)
            {
                this->logger_->log(this->episode, this->global_step, this->q_value_->getQValueAt(this->env_->reset(), 0)->norm_1(), (float)(clock() - t_begin) / CLOCKS_PER_SEC);
                this->do_log = false;
            }
            if (this->do_test_)
            {
                std::cout << "--------- TESTING ---------" << std::endl;
                this->do_test();
                this->do_test_ = false;
            }
        }
        std::cout << "Final QValue :" << *this->q_value_ << std::endl;
    }

    template <typename TObservation, typename TAction>
    void QLearning<TObservation, TAction>::do_test()
    {
    }

    template <typename TObservation, typename TAction>
    void QLearning<TObservation, TAction>::do_episode()
    {
        this->step = 0;
        this->episode += 1;
        this->current_obs = this->env_->reset();

        unsigned long stop_cond = this->global_step + this->planning_horizon_;
        while (this->global_step < stop_cond)
        {
            this->do_step();

            //Save the model
            this->do_save = (this->global_step % this->save_freq == 0);
            this->do_log = (this->global_step % this->log_freq == 0);
            this->do_test_ = (this->global_step % this->test_freq == 0);

            if (this->is_done)
            {
                break;
            }
        }
    }

    template <typename TObservation, typename TAction>
    void QLearning<TObservation, TAction>::do_step()
    {
        // Action selection following policy and exploration process
        auto current_action = this->select_action(this->current_obs);

        // One step in env and get next observation and rewards
        auto [next_obs, rewards, done] = this->env_->step(current_action);

        // Store experience in the associated buffer
        // this->experience->store_experience(this->current_obs, action, rewards, next_obs, done);

        this->last_obs = this->current_obs;
        this->current_obs = next_obs;
        this->is_done = done;

        // Update the model
        // this->update_model();
        double cval = this->q_value_->getQValueAt(this->last_obs, current_action, this->step);
        double expectval = this->discount_ * this->q_value_->getQValueAt(this->current_obs, this->step + 1)->max();
        double targetval = rewards[0];
        double target = targetval + expectval - cval;
        this->q_value_->updateQValueAt(this->last_obs, current_action, this->step, target);

        this->step++;
        this->global_step++;
    }

    template <typename TObservation, typename TAction>
    void QLearning<TObservation, TAction>::update_model()
    {
        // auto batch = this->experience_->sample(this->batch_size_);

        // for (const auto &transition : batch)
        // {
        //     auto [s, a, r, s_] = transition;
        //     target = r + this->discount_ * this->getQValueAt(s_, h + 1)->max() - this->getQValueAt(s, a, h);
        //     this->q_value_->updateQValueAt(s, a, h, target);
        // }

        // if (this->off_policy && t % this->target_update_freq == 0)
        // {
        //     this->update_target_model()
        // }
    }

    template <typename TObservation, typename TAction>
    TAction QLearning<TObservation, TAction>::select_action(const TObservation &obs)
    {
        // Do epsilon-greedy (si possible générique = EpsGreedy --|> Exploration)
        if ((rand() / double(RAND_MAX)) < this->exploration_process->getEpsilon())
        {
            return this->env_->getActionSpaceAt(obs)->sample();
        }
        else
        {
            return this->q_value_->getBestAction(obs, this->step);
        }
        // return this->exploration_->getAction(this->qvalue_, obs, this->step); // random is (tmp < epsilon) else qvalue(current_obs)
    }

} // namespace sdm

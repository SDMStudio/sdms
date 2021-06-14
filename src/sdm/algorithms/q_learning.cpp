#include <sdm/algorithms/q_learning.hpp>

namespace sdm
{

    QLearning::QLearning(std::shared_ptr<GymInterface> &env,
                                                std::shared_ptr<QValueFunction> q_value,
                                                std::shared_ptr<QValueFunction> q_target,
                                                std::shared_ptr<EpsGreedy> exploration,
                                                number horizon,
                                                double discount,
                                                double lr,
                                                double batch_size,
                                                unsigned long num_max_steps,
                                                std::string name) : env_(env),
                                                                    q_value_(q_value),
                                                                    q_target_(q_target),
                                                                    exploration_process(exploration),
                                                                    horizon_(horizon),
                                                                    discount_(discount),
                                                                    lr_(lr),
                                                                    batch_size_(batch_size),
                                                                    max_steps_(num_max_steps),
                                                                    name_(name)
    {
    }

    void QLearning::initLogger()
    {
        std::string format = "#> Episode : {}\tStep : {}/?\tQValue : {}\n";

        auto std_logger = std::make_shared<sdm::StdLogger>(format);
        auto file_logger = std::make_shared<sdm::FileLogger>(this->name_ + ".txt", format);
        auto csv_logger = std::make_shared<sdm::CSVLogger>(this->name_, std::vector<std::string>{"Episode", "Step", "QValue", "Time"});

        this->logger_ = std::make_shared<sdm::MultiLogger>(std::vector<std::shared_ptr<Logger>>{std_logger, file_logger, csv_logger});
    }

    void QLearning::do_initialize()
    {
        this->initLogger();

        this->q_value_->initialize();
        this->q_target_->initialize();
    }

    // std::shared_ptr<GymInterface> env, long nb_timesteps, number horizon, number test_freq, number save_freq, std::string save_folder, number verbose, long timestep_init, std::string log_file
    void QLearning::do_solve()
    {
        this->global_step = 0;
        std::cout << "-------- DO_SOLVE() ---------" << std::endl;
        std::cout << *this->q_value_ << std::endl;
        clock_t t_begin = clock();

        this->exploration_process->reset(this->max_steps_);

        while (this->global_step < this->max_steps_)
        {
            // Update exploration process
            this->exploration_process->update(this->global_step);

            // Do one episode
            this->do_episode();

            // Test current policy and write logs
            if (this->do_log_)
            {
                this->logger_->log(this->episode, this->global_step, this->q_value_->getQValuesAt(this->env_->reset(), 0)->max(), (float)(clock() - t_begin) / CLOCKS_PER_SEC);
                this->do_log_ = false;
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

    void QLearning::do_save()
    {
        this->q_value_->save(this->name_ + "_qvalue.bin");
    }

    void QLearning::do_test()
    {
    }

    void QLearning::do_episode()
    {
        this->step = 0;
        this->episode += 1;
        this->current_obs = this->env_->reset();

        unsigned long stop_cond = this->global_step + this->horizon_;
        while (this->global_step < stop_cond)
        {
            this->do_step();

            //Save the model
            this->do_save_ = (this->global_step % this->save_freq == 0);
            this->do_log_ = (this->global_step % this->log_freq == 0);
            this->do_test_ = (this->global_step % this->test_freq == 0);

            if (this->is_done)
            {
                break;
            }
        }
    }

    void QLearning::do_step()
    {
        // Action selection following policy and exploration process
        auto current_action = this->select_action(this->current_obs);

        // One step in env and get next observation and rewards
        // auto [next_obs, rewards, done] = this->env_->step(current_action);
        std::tuple<std::shared_ptr<Observation>, std::vector<double>, bool> feedback = this->env_->step(current_action);
        std::shared_ptr<Observation> next_obs = std::get<0>(feedback);
        double r = std::get<1>(feedback)[0];
        bool done = std::get<2>(feedback);

        // Store experience in the associated buffer
        // this->experience->store_experience(this->current_obs, action, rewards, next_obs, done);

        this->last_obs = this->current_obs;
        this->current_obs = next_obs;
        this->is_done = done;

        // Update the model
        // this->update_model();
        double cval = this->q_value_->getQValueAt(this->last_obs, current_action, this->step);
        double expectval = this->discount_ * this->q_value_->getQValuesAt(this->current_obs, this->step + 1)->max();
        double targetval = r;
        double target = targetval + expectval - cval;
        this->q_value_->updateQValueAt(this->last_obs, current_action, this->step, target);

        this->step++;
        this->global_step++;
    }

    void QLearning::update_model()
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

    std::shared_ptr<Action> QLearning::select_action(const std::shared_ptr<Observation> &obs)
    {
        // Do epsilon-greedy (si possible générique = EpsGreedy --|> Exploration)
        if ((rand() / double(RAND_MAX)) < this->exploration_process->getEpsilon())
        {
            return this->env_->getActionSpaceAt(obs, this->step)->sample()->toAction();
        }
        else
        {
            return this->q_value_->getBestAction(obs, this->step);
        }
        // return this->exploration_->getAction(this->qvalue_, obs, this->step); // random is (tmp < epsilon) else qvalue(current_obs)
    }

} // namespace sdm

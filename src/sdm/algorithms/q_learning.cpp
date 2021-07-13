#include <sdm/algorithms/q_learning.hpp>

namespace sdm
{

    QLearning::QLearning(std::shared_ptr<GymInterface> &env,
                         std::shared_ptr<ExperienceMemoryInterface> experience_memory,
                         std::shared_ptr<QValueFunction> q_value_table,
                         std::shared_ptr<QValueFunction> q_value_table_target,
                         std::shared_ptr<QValueBackupInterface> backup,
                         std::shared_ptr<EpsGreedy> exploration,
                         number horizon,
                         double discount,
                         double lr,
                         double batch_size,
                         unsigned long num_max_steps,
                         std::string name) : env_(env),
                                             experience_memory_(experience_memory),
                                             q_value_table_(q_value_table),
                                             q_value_table_target_(q_value_table_target),
                                             backup_(backup),
                                             exploration_process(exploration),
                                             horizon_(horizon),
                                             discount_(discount),
                                             lr_(lr),
                                             batch_size_(batch_size),
                                             max_steps_(num_max_steps),
                                             target_update_(1),
                                             name_(name)
    {
    }

    void QLearning::initLogger()
    {
        std::string format = "#> Episode : {}\tStep : {}/?\tValue : {}\n";

        auto std_logger = std::make_shared<sdm::StdLogger>(format);
        auto file_logger = std::make_shared<sdm::FileLogger>(this->name_ + ".txt", format);
        auto csv_logger = std::make_shared<sdm::CSVLogger>(this->name_, std::vector<std::string>{"Episode", "Step", "Value", "Time"});

        this->logger_ = std::make_shared<sdm::MultiLogger>(std::vector<std::shared_ptr<Logger>>{std_logger, file_logger, csv_logger});
    }

    void QLearning::do_initialize()
    {
        this->initLogger();

        this->q_value_table_->initialize();
        this->q_value_table_target_->initialize();
    }

    // std::shared_ptr<GymInterface> env, long nb_timesteps, number horizon, number test_freq, number save_freq, std::string save_folder, number verbose, long timestep_init, std::string log_file
    void QLearning::do_solve()
    {
        this->global_step = 0;
        this->episode = 0;
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
                this->logger_->log(this->episode, this->global_step, this->q_value_table_->getValueAt(this->env_->reset()->toState(), 0), (float)(clock() - t_begin) / CLOCKS_PER_SEC);
                this->do_log_ = false;
            }
            if (this->do_test_)
            {
                this->do_test();
                this->do_test_ = false;
            }
        }
        // std::cout << *this->q_value_table_ << std::endl;
        // this->q_value_table_->printNumberOfActions();
        // std::ofstream QValueStream(this->name_ + ".qvalue");
        // QValueStream << *this->q_value_table_ << std::endl;
        // QValueStream.close();

    }

    void QLearning::do_save()
    {
        this->q_value_table_->save(this->name_ + "_qvalue.bin");
    }

    void QLearning::do_test()
    {
    }

    void QLearning::do_episode()
    {   
        // std::cout << "-------- do_episode() ---------" << std::endl;
        // std::cout << *this->q_value_table_ << std::endl;
        // Le update marche pas, du coup pour le moment j'utilise le meme QVF pour le target depuis le debut
        // if (this->episode % target_update_ == 0)
        //     this->update_target();
        this->step = 0;
        this->episode += 1;
        this->observation = this->env_->reset();

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
        // std::cout << "-------- do_step() ---------" << std::endl;
        
        // Action selection following policy and exploration process
        this->action = this->select_action(this->observation);
        // One step in env and get next observation and rewards
        std::tie(this->next_observation, this->rewards_, this->is_done) = this->env_->step(this->action);
        // Push experience to memory
        this->experience_memory_->push(this->observation, this->action, this->rewards_[0], this->next_observation, this->step);
        // Backup and get Q Value Error
        double delta = this->backup_->backup(this->step);

        this->observation = this->next_observation;
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
        //     this->q_value_table_->updateQValueAt(s, a, h, target);
        // }

        // if (this->off_policy && t % this->target_update_freq == 0)
        // {
        //     this->update_target_model()
        // }
    }

    void QLearning::update_target()
    {
        *this->q_value_table_target_ = *this->q_value_table_;
    }

    std::shared_ptr<Action> QLearning::select_action(const std::shared_ptr<Observation> &observation)
    {
        // Do epsilon-greedy (si possible générique = EpsGreedy --|> Exploration)
        if (((rand() / double(RAND_MAX)) < this->exploration_process->getEpsilon()) || this->q_value_table_->isNotSeen(observation->toState(), this->step))
        {
            return this->env_->getActionSpaceAt(observation->toState(), this->step)->sample()->toAction();
        }
        else
        {
            return this->q_value_table_->getBestAction(observation->toState(), this->step);
        }
        // return this->exploration_->getAction(this->qvalue_, observation, this->step); // random is (tmp < epsilon) else qvalue(observation)
    }

} // namespace sdm

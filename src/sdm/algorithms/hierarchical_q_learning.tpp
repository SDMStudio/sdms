#include <sdm/algorithms/hierarchical_q_learning.hpp>

namespace sdm
{

    template <typename TObservation, typename TAction>
    HierarchicalQLearning<TObservation, TAction>::HierarchicalQLearning(std::shared_ptr<HierarchicalPrivateOccupancyMDP<TObservation, TAction>> &env,
                                                std::shared_ptr<HierarchicalMappedQValueFunction<Pair<TObservation, Joint<typename TObservation::ihistory_type>>, Joint<typename TAction::output>>> q_value_table,
                                                std::shared_ptr<HierarchicalMappedQValueFunction<Pair<TObservation, Joint<typename TObservation::ihistory_type>>, Joint<typename TAction::output>>> target_q_value,
                                                std::shared_ptr<EpsGreedy<TObservation, TAction>> exploration,
                                                number planning_horizon,
                                                double discount,
                                                double lr,
                                                double sf,
                                                double batch_size,
                                                unsigned long num_max_steps,
                                                std::string name) : env_(env),
                                                                    q_value_table_(q_value_table),
                                                                    target_q_value_table_(target_q_value),
                                                                    exploration_process(exploration),
                                                                    planning_horizon_(planning_horizon),
                                                                    discount_(discount),
                                                                    lr_(lr),
                                                                    sf_(sf),
                                                                    batch_size_(batch_size),
                                                                    max_steps_(num_max_steps),
                                                                    target_update_(1),
                                                                    name_(name)
    {
    }

    template <typename TObservation, typename TAction>
    void HierarchicalQLearning<TObservation, TAction>::initLogger()
    {
        std::string format = "#> Episode : {}\tStep : {}\tEpsilon : {}\tValue : {}\tQ Loss : {}\n";

        auto std_logger = std::make_shared<sdm::StdLogger>(format);
        auto file_logger = std::make_shared<sdm::FileLogger>(this->name_ + ".txt", format);
        auto csv_logger = std::make_shared<sdm::CSVLogger>(this->name_, std::vector<std::string>{"Episode", "Step", "Epsilon", "Value", "Q Loss", "Time"});

        this->logger_ = std::make_shared<sdm::MultiLogger>(std::vector<std::shared_ptr<Logger>>{std_logger, file_logger, csv_logger});
    }

    template <typename TObservation, typename TAction>
    void HierarchicalQLearning<TObservation, TAction>::do_initialize()
    {
        this->initLogger();

        // this->q_value_table_->initialize();
        // this->target_q_value_table_->initialize();

        this->episode = 0;
    }

    // std::shared_ptr<HierarchicalPrivateOccupancyMDP> env, long nb_timesteps, number planning_horizon, number test_freq, number save_freq, std::string save_folder, number verbose, long timestep_init, std::string log_file
    template <typename TObservation, typename TAction>
    void HierarchicalQLearning<TObservation, TAction>::do_solve()
    {
        this->global_step = 0;
        this->E_R = 0;
        std::cout << "-------- DO_SOLVE() ---------" << std::endl;
        std::cout << *this->q_value_table_ << std::endl;
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
                this->logger_->log(
                    this->episode, 
                    this->global_step, 
                    this->exploration_process->getEpsilon(),
                    this->E_R, 
                    this->q_loss,
                    (float)(clock() - t_begin) / CLOCKS_PER_SEC
                );
                this->do_log = false;
            }
            if (this->do_test_)
            {
                // std::cout << "--------- TESTING ---------" << std::endl;
                this->do_test();
                this->do_test_ = false;
            }
        }
        // std::cout << *this->q_value_table_ << std::endl;
    }

    template <typename TObservation, typename TAction>
    void HierarchicalQLearning<TObservation, TAction>::do_save()
    {
        this->q_value_table_->save(this->name_ + "_qvalue.bin");
    }

    template <typename TObservation, typename TAction>
    void HierarchicalQLearning<TObservation, TAction>::do_test()
    {
    }

    template <typename TObservation, typename TAction>
    void HierarchicalQLearning<TObservation, TAction>::do_episode()
    {   
        this->R = 0.0;
        this->q_loss = 0.0;
        this->step = 0;
        if (this->episode++ % this->target_update_ == 0){
            this->target_q_value_table_ = this->q_value_table_;
        }
        this->current_obs = this->env_->reset();

        unsigned long stop_cond = this->global_step + this->planning_horizon_;
        while (this->global_step < stop_cond)
        {
            this->do_step();

            //Save the model
            this->do_save_ = (this->global_step % this->save_freq == 0);
            this->do_log = (this->global_step % this->log_freq == 0);
            this->do_test_ = (this->global_step % this->test_freq == 0);

            if (this->is_done)
            {   
                this->E_R = this->E_R * this->sf_ + this->R * (1 - this->sf_);
                break;
            }
        }
    }

    template <typename TObservation, typename TAction>
    void HierarchicalQLearning<TObservation, TAction>::do_step()
    {
        // Action selection following policy and exploration process
        auto current_action = this->select_action(this->current_obs);
        auto jaction = this->env_->getJaction(current_action);
        // One step in env and get next observation and rewards
        auto [next_obs, rewards, done] = this->env_->step(current_action);

        auto r = rewards[0];

        // Store experience in the associated buffer
        // this->experience->store_experience(this->current_obs, action, rewards, next_obs, done);

        this->next_obs = next_obs;
        this->is_done = done;
        this->R += r;
        // Update the model
        // this->update_model();
        Joint<HistoryTree_p<number>> o = get_o(this->env_->previous_history_);
        Joint<HistoryTree_p<number>> next_o = get_o(this->env_->current_history_);
        double q_value = this->q_value_table_->getQValueAt(this->current_obs, o, jaction, this->step);
        double next_q_value = this->discount_ * this->q_value_table_->getNextValueAt(this->next_obs, next_o, this->step + 1);
        double target_q_value = r + next_q_value;
        double delta = target_q_value - q_value;
        this->q_loss += delta;

        // std::cout << "##> Step : " << this->step 
        //           << "\tJaction : " << jaction 
        //           << "\tQValue : " << q_value 
        //           << "\tReward : " << r 
        //           << "\tNextQValue : " << next_q_value 
        //           << "\tTargetQValue : " << target_q_value 
        //           << "\tDelta : " << delta 
        //           << "\n";

        this->q_value_table_->updateQValueAt(this->current_obs, o, jaction, this->step, delta);
        this->current_obs = next_obs;
        this->step++;
        this->global_step++;
    }

    template <typename TObservation, typename TAction>
    void HierarchicalQLearning<TObservation, TAction>::update_model()
    {
        // auto batch = this->experience_->sample(this->batch_size_);

        // for (const auto &transition : batch)
        // {
        //     auto [s, a, r, s_] = transition;
        //     delta = r + this->discount_ * this->getQValuesAt(s_, h + 1)->max() - this->getQValueAt(s, a, h);
        //     this->q_value_table_->updateQValueAt(s, a, h, delta);
        // }

        // if (this->off_policy && t % this->target_update_freq == 0)
        // {
        //     this->update_target_model()
        // }
    }

    template <typename TObservation, typename TAction>
    TAction HierarchicalQLearning<TObservation, TAction>::select_action(const TObservation &obs)
    {
        // Do epsilon-greedy (si possible générique = EpsGreedy --|> Exploration)
        if ((rand() / double(RAND_MAX)) < this->exploration_process->getEpsilon())
        {
            return this->env_->getActionSpaceAt(obs)->sample();
        }
        else
        {   
            return this->q_value_table_->getGreedyAction(std::make_pair(obs, get_o(this->env_->current_history_)), this->step);
        }
        // return this->exploration_->getAction(this->qvalue_, obs, this->step); // random is (tmp < epsilon) else qvalue(current_obs)
    }

    template <typename TObservation, typename TAction>
    Joint<HistoryTree_p<number>> HierarchicalQLearning<TObservation, TAction>::get_o(JointHistoryTree_p<number> history){
        // Number of agents.
        number N = this->env_->getUnderlyingProblem()->getNumAgents();
        // Construct the JointHistory.
        Joint<HistoryTree_p<number>> o;
        for (agent i = 0; i < N; i++)
        {
            o.push_back(history->at(i));
        }
        return o;
    }

} // namespace sdm

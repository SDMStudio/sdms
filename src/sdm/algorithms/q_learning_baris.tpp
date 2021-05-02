#include <sdm/algorithms/q_learning_baris.hpp>
#include <typeinfo>

namespace sdm
{

    template <typename TObservation, typename TAction>
    QLearningBaris<TObservation, TAction>::QLearningBaris(std::shared_ptr<JointHistoryPrivateOccupancyMDP<TObservation, TAction>> &env,
                                                std::shared_ptr<QValueFunction<TObservation, Joint<number>>> q_value,
                                                std::shared_ptr<QValueFunction<TObservation, Joint<number>>> q_target,
                                                std::shared_ptr<EpsGreedy<TObservation, TAction>> exploration,
                                                number planning_horizon,
                                                double discount,
                                                double lr,
                                                double batch_size,
                                                unsigned long num_max_steps,
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
    void QLearningBaris<TObservation, TAction>::initLogger()
    {
        std::string format = "#> Episode : {}\tStep : {}/?\tQValue : {}\n";

        auto std_logger = std::make_shared<sdm::StdLogger>(format);
        auto file_logger = std::make_shared<sdm::FileLogger>(this->name_ + ".txt", format);
        auto csv_logger = std::make_shared<sdm::CSVLogger>(this->name_, std::vector<std::string>{"Episode", "Step", "QValue", "Time"});

        this->logger_ = std::make_shared<sdm::MultiLogger>(std::vector<std::shared_ptr<Logger>>{std_logger, file_logger, csv_logger});
    }

    template <typename TObservation, typename TAction>
    void QLearningBaris<TObservation, TAction>::do_initialize()
    {
        this->initLogger();
        
        // std::cout << typeid(this->q_value_).name() << std::endl;
        this->q_value_->initialize();
        this->q_target_->initialize();
    }

    // std::shared_ptr<GymInterface> env, long nb_timesteps, number planning_horizon, number test_freq, number save_freq, std::string save_folder, number verbose, long timestep_init, std::string log_file
    template <typename TObservation, typename TAction>
    void QLearningBaris<TObservation, TAction>::do_solve()
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
                this->logger_->log(
                    this->episode, 
                    this->global_step, 
                    this->q_value_->getQValueAt(this->env_->reset(), 0)->max(), 
                    (float)(clock() - t_begin) / CLOCKS_PER_SEC
                );
                this->do_log = false;
            }
            // if (this->do_test_)
            // {
            //     std::cout << "--------- TESTING ---------" << std::endl;
            //     this->do_test();
            //     this->do_test_ = false;
            // }
        }
        // std::cout << "Final QValue :" << *this->q_value_ << std::endl;
    }

    template <typename TObservation, typename TAction>
    void QLearningBaris<TObservation, TAction>::do_test()
    {
    }

    template <typename TObservation, typename TAction>
    void QLearningBaris<TObservation, TAction>::do_episode()
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
    void QLearningBaris<TObservation, TAction>::do_step()
    {   
        // Action selection following policy and exploration process
        auto current_action = this->select_action(this->current_obs);
        // One step in env and get next observation and rewards
        auto [next_obs, rewards, done] = this->env_->step(current_action);
        // Store experience in the associated buffer
        // this->experience->store_experience(this->current_obs, action, rewards, next_obs, done);
        this->last_obs = this->current_obs;

        this->is_done = done;

        // Update the model
        // this->update_model();
        // i need to turn the current action to a joint action
        Joint<number> jaction;
        Joint<number> actions;
        actions.push_back(current_action.second);
        jaction.push_back(current_action.first.at(0)(std::make_pair(this->current_obs.second.at(0), actions)));
        jaction.push_back(current_action.second);
        double cval = this->q_value_->getQValueAt(this->last_obs, jaction, this->step);
        this->current_obs = next_obs;
        double expectval = this->discount_ * this->q_value_->getQValueAt(this->current_obs, this->step + 1)->max();
        double targetval = rewards[0];
        double target = targetval + expectval - cval;
        this->q_value_->updateQValueAt(this->last_obs, jaction, this->step, target);
        this->step++;
        this->global_step++;
    }

    template <typename TObservation, typename TAction>
    void QLearningBaris<TObservation, TAction>::update_model()
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
    TAction QLearningBaris<TObservation, TAction>::select_action(const TObservation &obs)
    {
        // Do epsilon-greedy (si possible générique = EpsGreedy --|> Exploration)
        if ((rand() / double(RAND_MAX)) < this->exploration_process->getEpsilon())
        {   
            return this->env_->getActionSpaceAt(obs)->sample();
        }
        else
        {
            return this->select_greedy_action(obs);


        }
        // return this->exploration_->getAction(this->qvalue_, obs, this->step); // random is (tmp < epsilon) else qvalue(current_obs)
    }

    template <typename TObservation, typename TAction>
    TAction QLearningBaris<TObservation, TAction>::select_greedy_action(const TObservation &obs)
    {
        PrivateOccupancyState<number, Joint<JointHistoryTree_p<number>>> s2 = obs.first;
        // Joint but it only has its own obs
        JointHistoryTree_p<number> o2 = obs.second.at(1);
        std::vector<Pair<JointHistoryTree_p<number>, Joint<number>>> acc_states;
        std::vector<number> n_actions;
        for (auto &o1: s2.getIndividualJointHistories(0)){
            Joint<JointHistoryTree_p<number>> o;
            o.push_back(o1);
            o.push_back(o2);
            for (const auto & u2: this->env_->dpomdp_->getActionSpace()->getSpace(1)->getAll()){
                Joint<typename TAction::first_type::value_type::output_type> actions;
                actions.push_back(u2);                
                auto acc_state = std::make_pair(o1, actions);
                acc_states.push_back(acc_state);
                auto state_q_func = std::make_pair(s2, o);
                auto qvalues = this->q_value_->getQValueAt(state_q_func, this->step);
                std::map<Joint<typename TAction::first_type::value_type::output_type>, double> q_values_u;
                for(const auto & u1_: this->env_->dpomdp_->getActionSpace()->getSpace(0)->getAll()){
                    for(const auto & u2_: this->env_->dpomdp_->getActionSpace()->getSpace(1)->getAll()){
                        Joint<typename TAction::first_type::value_type::output_type> u;
                        u.push_back(u1_);
                        u.push_back(u2_);
                        if (u2 != u2_){
                            q_values_u[u] = std::numeric_limits<double>::lowest();
                        } else {
                            q_values_u[u] = qvalues->at(u);
                        }
                    }
                }
                // auto greedy_u = q_values_u->argmax();
                auto greedy_u_it  = std::max_element(
                    q_values_u.begin(), 
                    q_values_u.end(),
                    [](const std::pair<Joint<typename TAction::first_type::value_type::output_type>, double>& p1, 
                       const std::pair<Joint<typename TAction::first_type::value_type::output_type>, double>& p2
                      )
                       {
                        return p1.second < p2.second; 
                       }
                );
                Joint<number> greedy_u = greedy_u_it->first;
                number greedy_u1 = greedy_u.at(0);
                n_actions.push_back(greedy_u1);
            }
        }
        DeterministicDecisionRule<Pair<JointHistoryTree_p<number>, Joint<number>>, number> a1(acc_states, n_actions);

        std::map<typename TAction::first_type::value_type::output_type, double> q_values_u2;
        for(const auto & u2: this->env_->dpomdp_->getActionSpace()->getSpace(1)->getAll()){
            q_values_u2[u2] = 0;
            Joint<typename TAction::first_type::value_type::output_type> actions;
            actions.push_back(u2);
            // o only has o1 btw, not o2. o2 is public from the start.
            for(auto &x_o__prob: s2){
                JointHistoryTree_p<number> o1 = x_o__prob.first.second.at(0);
                if (o1->getIndividualHistory(1) != o2->getIndividualHistory(0)){
                    continue;
                }
                Joint<JointHistoryTree_p<number>> o;
                o.push_back(o1);
                o.push_back(o2);
                auto acc_state = std::make_pair(o1, actions);
                number u1 = a1(acc_state);
                Joint<typename TAction::first_type::value_type::output_type> jactions;
                jactions.push_back(u1);
                jactions.push_back(u2);
                auto state_q_func = std::make_pair(s2, o);
                q_values_u2[u2] += x_o__prob.second * this->q_value_->getQValueAt(state_q_func, jactions, this->step);
            }
        }
        // number best_u2 = qvalues->argmax();
        auto greedy_u2_it  = std::max_element(
            q_values_u2.begin(), 
            q_values_u2.end(),
            [](const std::pair<typename TAction::first_type::value_type::output_type, double>& p1, 
               const std::pair<typename TAction::first_type::value_type::output_type, double>& p2
              )
               {
                  return p1.second < p2.second; 
               }
        );
        number greedy_u2 = greedy_u2_it->first;

        Joint<DeterministicDecisionRule<Pair<JointHistoryTree_p<number>, Joint<number>>, number>> jddr;
        jddr.push_back(a1);
        return std::make_pair(jddr, greedy_u2);
    }

} // namespace sdm

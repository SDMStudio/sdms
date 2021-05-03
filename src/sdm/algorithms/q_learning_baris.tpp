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
        // We turn current_action which is a TAction to Joint<number>.
        Joint<number> jaction;
        Joint<number> actions_rev;
        actions_rev.push_back(current_action.second);
        for (int agent = this->env_->dpomdp_->getNumAgents() - 2; agent >= 0; agent--)
        {   
            auto p_ihist = this->current_obs.second.at(agent);
            auto idr = current_action.first.at(agent);
            actions_rev.push_back(idr(std::make_pair(p_ihist, actions_rev)));
        }
        for (int i = actions_rev.size() - 1; i >= 0; i--)
        {
            jaction.push_back(actions_rev[i]);
        }
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
        // Number of agents.
        number N = this->env_->dpomdp_->getNumAgents();
        // This has all histories from 0 to N-1, so all except N, and probability for each ofc.
        PrivateOccupancyState<number, Joint<JointHistoryTree_p<number>>> sN = obs.first;
        // Joint but it only has its own obsrvations since it's history of agent N, the one that sees none of the other agents.
        JointHistoryTree_p<number> oN = obs.second.at(N - 1);
        // Ddr for all agents except N.
        Joint<DeterministicDecisionRule<Pair<JointHistoryTree_p<number>, Joint<number>>, number>> jddr;

        // For all agents starting with 0 to N-2 (1 to N-1), we need to calculate their ddr.
        for (number agent = 0; agent < N - 1; agent++){
            // The last element, which will be at index agent, will be the vector of all possible jactions of previous agents.
            std::vector<
                std::vector<
                    typename TAction::first_type::value_type::input_type::second_type>> vec_vec_prev_actions_rev = this->env_
                        ->get_vec_vec_prev_actions_rev(agent);
            // Accessible states for the idr of agent.
            std::vector<Pair<JointHistoryTree_p<number>, Joint<number>>> acc_states;
            // Actions for each of these for agent (idk why it's called like that, using the same name as the constructor of ddr.)
            std::vector<number> n_actions;
            // For each possible histories for agent:
            for (auto &oAgent: sN.getIndividualJointHistories(agent)){
                // For each possible previous actions:
                for (const auto & prev_actions_rev: vec_vec_prev_actions_rev[agent]){
                    // State for idr of agent.
                    auto acc_state = std::make_pair(oAgent, prev_actions_rev);
                    // Push it.
                    acc_states.push_back(acc_state);
                    // Q values for actions of Agent.
                    std::map<typename TAction::first_type::value_type::output_type, double> q_values_uAgent;
                    // For each possible uAgent:
                    for(const auto & uAgent: this->env_->dpomdp_->getActionSpace()->getSpace(agent)->getAll()){
                        // Initialize it to 0.
                        q_values_uAgent[uAgent] = 0;
                        // For each support (?) of sN:
                        for(auto &x_o__prob: sN){
                            // actions_rev is initialized with actions of agents N, N-1, ..., Agent - 1.
                            Joint<typename TAction::first_type::value_type::output_type> actions_rev = prev_actions_rev;
                            // Add action of Agent.
                            actions_rev.push_back(uAgent);
                            // Is this support (?) compatible with the public history?
                            bool compatible = true;
                            // Initialize Joint History.
                            Joint<JointHistoryTree_p<number>> o;
                            // For agent_s from 0 to Agent - 1 (in other words agent_s who are not visible for agent):
                            for (number agent_ = 0; agent_ < agent - 1; agent_++){
                                // Get history of agent_.
                                JointHistoryTree_p<number> oAgent_ = x_o__prob.first.second.at(agent_);
                                // If this isn't the case that this support (?) isn't compatible.
                                if (oAgent_->getIndividualHistory(N - 1 - agent_) != oN->getIndividualHistory(0)){
                                    compatible = false;
                                }
                                // Push history of agent_ to the Joint History.
                                o.push_back(oAgent_);
                            }
                            // If this support (?) isn't compatible then we skip to the next one.
                            if (!(compatible)){
                                continue;
                            }
                            // For each agent_ who is visible for agent (that is agent_s between [agent+1 and N-1 (N)]):
                            for (number agent_ = agent; agent_ < N; agent_++){
                                // Get its history from the true history.
                                JointHistoryTree_p<number> oAgent_ = obs.second.at(agent);
                                // Push it to the Joint History.
                                o.push_back(oAgent_);
                            }
                            // Construct the key to the Q table.
                            auto key = std::make_pair(sN, o);
                            // For each agent_ between [agent - 1 and 0], that is agent_s that agent cannot observe:
                            for (int agent_ = agent - 1; agent_ >= 0; agent_--){
                                // Get a possible history for the agent_.
                                JointHistoryTree_p<number> oAgent_ = x_o__prob.first.second.at(agent_);
                                // Cretate the entry to the idr of agent_.
                                auto acc_state_ = std::make_pair(oAgent_, actions_rev);
                                // Get agent_'s idr.
                                auto aAgent_ = jddr.at(agent_);
                                // Get uAgent_ given aAgent_ and acc_state_.
                                number uAgent_ = aAgent_(acc_state_);
                                // Push uAgent_ to actions_rev.
                                actions_rev.push_back(uAgent_);
                            }
                            // Initialize the Joint Action.
                            Joint<typename TAction::first_type::value_type::output_type> jaction;
                            // Correctly push the actions to it.
                            for (int i = actions_rev.size() - 1; i >= 0; i--){
                                jaction.push_back(actions_rev[i]);
                            }
                            // Increase uAgent's Q value given all that and probability of this support (?).
                            q_values_uAgent[uAgent] += x_o__prob.second * this->q_value_->getQValueAt(key, jaction, this->step);
                        }
                    }
                    // number greedy_uAgent = q_values_uAgent->argmax();
                    auto greedy_uAgent_it  = std::max_element(
                        q_values_uAgent.begin(), 
                        q_values_uAgent.end(),
                        [](const std::pair<typename TAction::first_type::value_type::output_type, double>& p1, 
                        const std::pair<typename TAction::first_type::value_type::output_type, double>& p2
                        )
                        {
                            return p1.second < p2.second; 
                        }
                    );
                    number greedy_uAgent = greedy_uAgent_it->first;
                    // We have found the greedy_uAgent for this key, so we push it here.
                    n_actions.push_back(greedy_uAgent);
                }
            }
            // Initialize idr for Agent.
            DeterministicDecisionRule<Pair<JointHistoryTree_p<number>, Joint<number>>, number> aAgent(acc_states, n_actions);
            // Push it to the jddr.
            jddr.push_back(aAgent);
        }        

        // Q values for actions of Agent N.
        std::map<typename TAction::first_type::value_type::output_type, double> q_values_uN;
        // For each possible uN:
        for(const auto & uN: this->env_->dpomdp_->getActionSpace()->getSpace(N - 1)->getAll()){
            // Initialize it to 0.
            q_values_uN[uN] = 0;
            // For each support (?) of sN:
            for(auto &x_o__prob: sN){
                // This will contain all actions of agents starting with Agent N until Agent 0.
                Joint<typename TAction::first_type::value_type::output_type> actions_rev;
                // Push uN.
                actions_rev.push_back(uN);
                // Is this support (?) compatible with the public history?
                bool compatible = true;
                // Initialize Joint History.
                Joint<JointHistoryTree_p<number>> o;
                // For agents from 0 to N-2 (1 to N-1)
                for (number agent_ = 0; agent_ < N - 1; agent_++){
                    // Get history of agent_.
                    JointHistoryTree_p<number> oAgent_ = x_o__prob.first.second.at(agent_);
                    // If this isn't the case that this support (?) isn't compatible.
                    if (oAgent_->getIndividualHistory(N - 1 - agent_) != oN->getIndividualHistory(0)){
                        compatible = false;
                    }
                    // Push history of agent_ to the Joint History.
                    o.push_back(oAgent_);
                }
                // If this support (?) isn't compatible then we skip to the next one.
                if (!(compatible)){
                    continue;
                }
                // Push oN (all the others were already pushed.).
                o.push_back(oN);
                // Construct the key to the Q table.
                auto key = std::make_pair(sN, o);
                // For each agent_ between [N - 1 and 0], that is agent_s that agent N cannot observe: (same thing as above...)
                for (int agent_ = N - 2; agent_ >= 0; agent_--){
                    // Get a possible history for the agent_.
                    JointHistoryTree_p<number> oAgent_ = x_o__prob.first.second.at(agent_);
                    // Cretate the entry to the idr of agent_.
                    auto acc_state_ = std::make_pair(oAgent_, actions_rev);
                    // Get agent_'s idr.
                    auto aAgent_ = jddr.at(agent_);
                    // Get uAgent_ given aAgent_ and acc_state_.
                    number uAgent_ = aAgent_(acc_state_);
                    // Push uAgent_ to actions_rev.
                    actions_rev.push_back(uAgent_);
                }
                // Push uN (all the others were already pushed.).
                actions_rev.push_back(uN);
                // Initialize the Joint Action.
                Joint<typename TAction::first_type::value_type::output_type> jaction;
                // Correctly push the actions to it.
                for (int i = actions_rev.size() - 1; i >= 0; i--){
                    jaction.push_back(actions_rev[i]);
                }
                // Increase uN's Q value given all that and probability of this support (?).
                q_values_uN[uN] += x_o__prob.second * this->q_value_->getQValueAt(key, jaction, this->step);
            }
        }

        // number greedy_uN = q_values_uN->argmax();
        auto greedy_uN_it  = std::max_element(
            q_values_uN.begin(), 
            q_values_uN.end(),
            [](const std::pair<typename TAction::first_type::value_type::output_type, double>& p1, 
               const std::pair<typename TAction::first_type::value_type::output_type, double>& p2
              )
               {
                  return p1.second < p2.second; 
               }
        );
        number greedy_uN = greedy_uN_it->first;
        // Return the Pair<Joint<DeterministicDecisionRule<Pair<JointHistoryTree_p<number>, Joint<number>>, number>>, number> object.
        return std::make_pair(jddr, greedy_uN);
    }

} // namespace sdm

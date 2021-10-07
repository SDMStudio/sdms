#include <sdm/algorithms/q_learning.hpp>
#include <sdm/world/occupancy_mdp.hpp>

namespace sdm
{

    QLearning::QLearning(std::shared_ptr<GymInterface> &env,
                                 std::shared_ptr<ExperienceMemoryInterface> experience_memory,
                                 std::shared_ptr<QValueFunction> q_value,
                                 std::shared_ptr<QValueFunction> q_target,
                                 std::shared_ptr<QValueBackupInterface> backup,
                                 std::shared_ptr<EpsGreedy> exploration,
                                 number horizon,
                                 double discount,
                                 double lr,
                                 double smooth,
                                 unsigned long num_episodes,
                                 std::string name) : Algorithm(name),
                                                     env_(env),
                                                     experience_memory_(experience_memory),
                                                     q_value_(q_value),
                                                     q_target_(q_target),
                                                     backup_(backup),
                                                     exploration_process(exploration),
                                                     horizon_(horizon),
                                                     discount_(discount),
                                                     lr_(lr),
                                                     num_episodes_(num_episodes)
    {
    }

    void QLearning::initLogger()
    {
        std::string format = "#> Episode : {}\tStep : {}/?\tEpsilon : {}\tValue : {}\tT(s) : {}\tN(S) : {}\n";

        auto std_logger = std::make_shared<sdm::StdLogger>(format);
        auto file_logger = std::make_shared<sdm::FileLogger>(name_ + ".txt", format);
        auto csv_logger = std::make_shared<sdm::CSVLogger>(name_, std::vector<std::string>{"Episode", "Step", "Epsilon", "Value", "Time", "N(S)"});

        logger_ = std::make_shared<sdm::MultiLogger>(std::vector<std::shared_ptr<Logger>>{std_logger, file_logger, csv_logger});
    }

    void QLearning::initialize()
    {
        initLogger();
        q_value_->initialize();
        q_target_->initialize();
        exploration_process->reset(num_episodes_);
        global_step = 0;
        episode = 0;
    }

    void QLearning::solve()
    {
        t_begin = clock();

        // Run N episodes
        while (episode < num_episodes_)
        {
            // Update exploration process at each episode
            exploration_process->update(episode);

            // Do one episode
            doEpisode();
        }
    }

    void QLearning::doEpisode()
    {
        observation = getEnv()->reset(); // Reset the environment

        number max_num_steps_by_ep = (horizon_ > 0) ? horizon_ : max_num_steps_by_ep_;
        unsigned long last_step = global_step + max_num_steps_by_ep; // Compute the last step of the episode

        step = 0;
        while (!is_done && global_step < last_step)
        {
            // Do one step
            doStep();
        }
        endEpisode();
    }

    void QLearning::endEpisode()
    {
        // Increment episode
        episode++;

        // Test current policy and write logs
        if (do_log_)
        {
            // Log data on the output stream
            logger_->log(episode, global_step, exploration_process->getEpsilon(), backup_->getValueAt(getEnv()->reset()->toState(), 0), (float)(clock() - t_begin) / CLOCKS_PER_SEC, q_value_->getNumStates());
            do_log_ = false;
        }
        if (do_test_)
        {
            test();
            do_test_ = false;
        }
    }

    void QLearning::doStep()
    {
        // Action selection following policy and exploration process
        auto action = this->selectAction(this->observation, this->step);

        // Execute one step in env and get next observation and rewards
        auto [next_observation, rewards, is_done] = this->getEnv()->step(action);
        this->is_done = is_done;

        // Compute next greedy action
        auto next_greedy_action = this->selectGreedyAction(next_observation, this->step + 1);

        // Push experience to memory
        this->experience_memory_->push(this->observation, action, rewards[0], next_observation, next_greedy_action, this->step);

        this->observation = next_observation;

        // Backup and get Q Value Error
        this->backup_->update(this->step);

        endStep();
    }

    void QLearning::endStep()
    {
        // Increment step
        step++;
        global_step++;

        // Save the model
        do_save_ = (global_step % save_freq == 0);
        do_log_ = (global_step % log_freq == 0);
        do_test_ = (global_step % test_freq == 0);
    }

    void QLearning::save()
    {
        q_value_->save(name_ + "_qvalue.bin");
    }

    void QLearning::test()
    {
    }

    void QLearning::updateTarget()
    {
        *q_target_ = *q_value_;
    }

    std::shared_ptr<Action> QLearning::selectAction(const std::shared_ptr<Observation> &observation, number t)
    {
        // If sampled value is lower than epsilon
        if ((rand() / double(RAND_MAX)) < exploration_process->getEpsilon())
        {
            // Get random action
            return getEnv()->getRandomAction(observation, t);
        }
        else
        {
            // Get greedy action
            return this->selectGreedyAction(observation, t);
        }
    }

    std::shared_ptr<GymInterface> QLearning::getEnv() const
    {
        return this->env_;
    }

    std::shared_ptr<Action> QLearning::selectGreedyAction(const std::shared_ptr<Observation> &observation, number t)
    {
        std::shared_ptr<Action> greedy_action;
        double best_value = -std::numeric_limits<double>::max(), tmp;
        
        auto action_space = getEnv()->getActionSpaceAt(observation, t);
        for (auto action : *action_space)
        {
            if (best_value < (tmp = this->q_value_->getQValueAt(observation->toState(), action->toAction(), t)))
            {
                best_value = tmp;
                greedy_action = action->toAction();
            }
        }
        return greedy_action;
    }

} // namespace sdm

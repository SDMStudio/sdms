#include <sdm/algorithms/q_learning.hpp>
#include <sdm/world/occupancy_mdp.hpp>

namespace sdm
{

    template <class TInput>
    QLearning<TInput>::QLearning(std::shared_ptr<GymInterface> &env,
                                 std::shared_ptr<ExperienceMemoryInterface> experience_memory,
                                 std::shared_ptr<QValueFunction<TInput>> q_value,
                                 std::shared_ptr<QValueFunction<TInput>> q_target,
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

    template <class TInput>
    void QLearning<TInput>::initLogger()
    {
        std::string format = "#> Episode : {}\tStep : {}/?\tEpsilon : {}\tValue : {}\tT(s) : {}\tN(S) : {}\n";

        auto std_logger = std::make_shared<sdm::StdLogger>(format);
        auto file_logger = std::make_shared<sdm::FileLogger>(name_ + ".txt", format);
        auto csv_logger = std::make_shared<sdm::CSVLogger>(name_, std::vector<std::string>{"Episode", "Step", "Epsilon", "Value", "Time", "N(S)"});

        logger_ = std::make_shared<sdm::MultiLogger>(std::vector<std::shared_ptr<Logger>>{std_logger, file_logger, csv_logger});
    }

    template <class TInput>
    void QLearning<TInput>::initialize()
    {
        initLogger();
        q_value_->initialize();
        q_target_->initialize();
        exploration_process->reset(num_episodes_);
        global_step = 0;
        episode = 0;
    }

    template <class TInput>
    void QLearning<TInput>::solve()
    {
        t_begin = clock();

        // Run N episodes
        while (episode < num_episodes_)
        {
            // Update exploration process at each episode
            exploration_process->update(episode);

            // Do one episode
            doEpisode();
            episode++;

            // Test current policy and write logs
            if (do_log_)
            {
                logger_->log(episode, global_step, exploration_process->getEpsilon(), backup_->getValueAt(getEnv()->reset()->toState(), 0), (float)(clock() - t_begin) / CLOCKS_PER_SEC, q_value_->getNumStates());
                do_log_ = false;
            }
            if (do_test_)
            {
                test();
                do_test_ = false;
            }
        }
    }

    template <class TInput>
    void QLearning<TInput>::save()
    {
        q_value_->save(name_ + "_qvalue.bin");
    }

    template <class TInput>
    void QLearning<TInput>::test()
    {
    }

    template <class TInput>
    void QLearning<TInput>::doEpisode()
    {
        observation = getEnv()->reset(); // Reset the environment

        number max_num_steps_by_ep = (horizon_ > 0) ? horizon_ : max_num_steps_by_ep_;
        unsigned long last_step = global_step + max_num_steps_by_ep; // Compute the last step of the episode

        step = 0;
        while (!is_done && global_step < last_step)
        {
            // Do one step
            doStep();
            step++;
            global_step++;

            // Save the model
            do_save_ = (global_step % save_freq == 0);
            do_log_ = (global_step % log_freq == 0);
            do_test_ = (global_step % test_freq == 0);
        }
    }

    template <class TInput>
    void QLearning<TInput>::doStep()
    {
        // Action selection following policy and exploration process
        auto action = selectAction(observation, step);

        // Execute one step in env and get next observation and rewards
        auto [next_observation, rewards_, done] = getEnv()->step(action);
        this->is_done = done;

        // Compute next greedy action
        auto next_greedy_action = selectGreedyAction(next_observation, step + 1);

        // Push experience to memory
        experience_memory_->push(observation, action, rewards_[0], next_observation, next_greedy_action, step);

        observation = next_observation;

        // Backup and get Q Value Error
        double delta = backup_->update(step);
    }

    template <class TInput>
    void QLearning<TInput>::updateTarget()
    {
        *q_target_ = *q_value_;
    }

    template <class TInput>
    std::shared_ptr<Action> QLearning<TInput>::selectAction(const std::shared_ptr<Observation> &observation, number t)
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
            auto a = selectGreedyAction(observation->toState(), t);
            return (a) ? a : getEnv()->getRandomAction(observation, t);
        }
    }

    template <class TInput>
    std::shared_ptr<Action> QLearning<TInput>::selectGreedyAction(const std::shared_ptr<Observation> &observation, number t)
    {
        return backup_->getGreedyAction(observation->toState(), t);
    }

    template <class TInput>
    std::shared_ptr<GymInterface> QLearning<TInput>::getEnv() const
    {
        return this->env_;
    }

    // template <class TInput>
    // void QLearning<TInput>::saveResults(std::string filename, double other)
    // {
    //     std::ofstream ofs;
    //     ofs.open(filename, std::ios::out | std::ios::app);
    //     ofs << other << ",";
    //     ofs << backup_->getValueAt(getEnv()->reset()->toState(), 0) << ",";
    //     ofs << std::static_pointer_cast<OccupancyMDP>(getEnv())->getMDPGraph()->getNumNodes() << ",";
    //     // number num_max_jhist = 0, tmp;
    //     // for (const auto &state : std::static_pointer_cast<OccupancyMDP>(getEnv())->getStoredStates())
    //     // {
    //     //     if (num_max_jhist < (tmp = state->toOccupancyState()->getJointHistories().size()))
    //     //     {
    //     //         num_max_jhist = tmp;
    //     //     }
    //     // }
    //     // ofs << num_max_jhist << ",";
    //     ofs << ((float)(clock() - t_begin) / CLOCKS_PER_SEC);

    //     ofs << "\n";
    //     ofs.close();
    // }

} // namespace sdm

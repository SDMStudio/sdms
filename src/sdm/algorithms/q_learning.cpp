#include <sdm/types.hpp>
#include <sdm/algorithms/q_learning.hpp>
#include <sdm/world/belief_mdp.hpp>

namespace sdm
{

    QLearning::QLearning(const std::shared_ptr<GymInterface> &env,
                         std::shared_ptr<ExperienceMemoryInterface> experience_memory,
                         std::shared_ptr<QValueFunction> q_value,
                         std::shared_ptr<QValueFunction> q_target,
                         std::shared_ptr<EpsGreedy> exploration,
                         number horizon,
                         double smooth,
                         unsigned long num_episodes,
                         std::string name) : Algorithm(name),
                                             env_(env),
                                             experience_memory_(experience_memory),
                                             q_value_(q_value),
                                             q_target_(q_target),
                                             exploration_process(exploration),
                                             horizon_(horizon),
                                             num_episodes_(num_episodes)
    {
    }

    void QLearning::initLogger()
    {
        // ************* Global Logger ****************
        // Text Format for standard output stream
        std::string format = config::LOG_SDMS + "Episode {:>8}/" + std::to_string(this->num_episodes_) + "    Step {:<14} Value {:<12.4f} T(s) {:<12.4f} Eps {:<6.2f}";

        // Titles of logs
        std::vector<std::string> list_logs{"Episode", "Step", "Value", "Time", "Epsilon"};

        // Specific logs for belief MDPs
        if (sdm::isInstanceOf<BeliefMDPInterface>(getEnv()))
        {
            format = format + " NumState {:<8}";
            list_logs.push_back("NumState");
        }
        format = format + "\n";

        // Build a logger that prints logs on the standard output stream
        auto std_logger = std::make_shared<sdm::StdLogger>(format);

        // Build a logger that stores data in a CSV file
        auto file_logger = std::make_shared<sdm::FileLogger>(this->getName() + ".txt", format);

        // Build a multi logger that combines previous loggers
        auto csv_logger = std::make_shared<sdm::CSVLogger>(this->getName(), list_logs);

        // Build a multi logger that combines previous loggers
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

    void QLearning::logging()
    {

        if (auto derived = std::dynamic_pointer_cast<BeliefMDPInterface>(getEnv()))
        {
            // Print in loggers some execution variables
            this->logger_->log(this->episode,
                               this->global_step,
                               this->q_value_->getValueAt(getEnv()->reset(), 0),
                               getExecutionTime(),
                               this->exploration_process->getEpsilon(),
                               derived->getMDPGraph()->getNumNodes());
        }
        else
        {
            // Print in loggers some execution variables
            this->logger_->log(this->episode,
                               this->global_step,
                               this->q_value_->getValueAt(getEnv()->reset(), 0),
                               getExecutionTime(),
                               this->exploration_process->getEpsilon());
        }
    }

    void QLearning::solve()
    {

        printStartInfo();
        startExecutionTime();

        // Run N episodes
        while (episode < num_episodes_)
        {
            // Update exploration process at each episode
            exploration_process->update(episode);

            // Do one episode
            doEpisode();
        }
        printEndInfo();
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
        this->is_done = false;

        // Test current policy and write logs
        if (do_log_)
        {
            // Log data on the output stream
            logging();
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
        auto [next_greedy_action, _] = this->q_value_->getGreedyActionAndValue(next_observation, this->step + 1);

        // Push experience to memory
        this->experience_memory_->push(this->observation, action, rewards[0], next_observation, next_greedy_action, this->step);

        this->observation = next_observation;

        // Backup and get Q Value Error
        this->q_value_->updateValueAt(this->step);

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
        q_value_->save(this->getName() + "_qvalue.bin");
    }

    void QLearning::test()
    {
    }

    void QLearning::updateTarget()
    {
        *q_target_ = *q_value_;
    }

    std::shared_ptr<Action> QLearning::selectAction(const std::shared_ptr<State> &state, number t)
    {
        // If sampled value is lower than epsilon
        if ((rand() / double(RAND_MAX)) < exploration_process->getEpsilon())
        {
            // Get random action
            return getEnv()->getRandomAction(state, t);
        }
        else
        {
            // Get greedy action
            return this->q_value_->getGreedyAction(state, t);
        }
    }

    std::shared_ptr<GymInterface> QLearning::getEnv() const
    {
        return this->env_;
    }

    std::string QLearning::getAlgorithmName()
    {
        return "QLearning";
    }

    void QLearning::printStartInfo()
    {
        std::cout << config::SDMS_THEME_1 << "------------------------------------" << std::endl;
        std::cout << config::LOG_SDMS << "START LEARNING (" << this->getAlgorithmName() << ")" << std::endl;
        std::cout << config::SDMS_THEME_1 << "------------------------------------" << config::NO_COLOR << std::endl;
    }

    void QLearning::printEndInfo()
    {
        std::cout << config::SDMS_THEME_1 << "------------------------------------" << std::endl;
        std::cout << config::LOG_SDMS << "END LEARNING (" << this->getAlgorithmName() << ")" << std::endl;
        std::cout << config::SDMS_THEME_1 << "------------------------------------" << config::NO_COLOR << std::endl;
    }

} // namespace sdm

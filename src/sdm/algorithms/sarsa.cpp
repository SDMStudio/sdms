#include <sdm/algorithms/sarsa.hpp>
#include <sdm/core/action/random_decision_rule.hpp>

#include <sdm/world/belief_mdp.hpp>
#include <sdm/utils/value_function/qfunction/pwlc_qvalue_function.hpp>

namespace sdm
{
    SARSA::SARSA(const std::shared_ptr<GymInterface> &env,
                 std::shared_ptr<ExperienceMemoryInterface> experience_memory,
                 std::shared_ptr<QValueFunction> q_value,
                 std::shared_ptr<QValueFunction> q_target,
                 std::shared_ptr<EpsGreedy> exploration,
                 number horizon,
                 double rate_start,
                 double rate_end,
                 double rate_decay,
                 unsigned long num_episodes,
                 std::string name) : QLearning(env, experience_memory, q_value, q_target, exploration, horizon, rate_start, rate_end, rate_decay, num_episodes, name)
    {
    }

    void SARSA::initialize()
    {
        QLearning::initialize();
        this->current = std::numeric_limits<double>::lowest();
        this->cumul_reward = 0.0;
        for (int t = 0; t < this->horizon_; t++)
        {
            this->actors.push_back(this->default_decision_rule(t));
        }
        this->actors.push_back(this->default_decision_rule(this->horizon_ - 1));
    }

    void SARSA::doEpisodeRecursive(const std::shared_ptr<State> &obs, number t)
    {
        if ((this->is_done) || (t == this->horizon_))
            return;

        // Action selection following policy and exploration process
        auto action = this->selectAction(obs, t);

        // Execute one step in env and get next observation and rewards
        auto [next_observation, rewards, is_done] = this->getEnv()->step(action);
        this->is_done = is_done;

        this->cumul_reward += pow(/* this->getEnv()->getDiscount() */ 1, t) * rewards[0];

        // Compute next greedy action
        auto [next_greedy_action, _] = this->q_value_->getGreedyActionAndValue(next_observation, t + 1);

        // Push experience to memory
        this->experience_memory_->push(obs, action, rewards[0], next_observation, next_greedy_action, t);

        this->doEpisodeRecursive(next_observation, t + 1);

        // greedy joint decision rule
        if (this->cumul_reward >= this->current)
        {
            this->actors[t] = action;
            this->current = this->cumul_reward;
        }

        // Backup and get Q Value Error
        this->q_value_->updateValueAt(this->learning_rate, t);

        endStep();
    }

    void SARSA::endEpisode()
    {
        QLearning::endEpisode();

        this->current = (this->current < this->cumul_reward) ? this->cumul_reward : this->current;
        this->cumul_reward = 0.0;
    }

    std::shared_ptr<Action> SARSA::default_decision_rule(number t) const
    {
        auto solvable_by_dp = std::dynamic_pointer_cast<SolvableByDP>(this->getEnv());
        return std::make_shared<RandomDecisionRule>(solvable_by_dp->getUnderlyingProblem()->getActionSpace(t));
    }

    std::string SARSA::getAlgorithmName()
    {
        return "SARSA";
    }

    void SARSA::initLogger()
    {
        // ************* Global Logger ****************
        // Text Format for standard output stream
        std::string format = "\r" + config::LOG_SDMS + "Episode {:>8}/" + std::to_string(this->num_episodes_) + "    Step {:<14} Value {:<12.4f} VPolicy {:<12.4f} T(s) {:<12.4f} Eps {:<6.2f} LR {:<6.2f}";

        // Titles of logs
        std::vector<std::string> list_logs{"Episode", "Step", "Value", "VPolicy", "Time", "Epsilon", "LearningRate"};

        // Specific logs for belief MDPs
        if (sdm::isInstanceOf<BeliefMDPInterface>(getEnv()))
        {
            format = format + " NumState {}";
            list_logs.push_back("NumState");
        }
        format = format + "";

        // Build a logger that prints logs on the standard output stream
        auto std_logger = std::make_shared<sdm::StdLogger>(format);

        // Build a logger that stores data in a CSV file
        auto file_logger = std::make_shared<sdm::FileLogger>(this->getName() + ".txt", format);

        // Build a multi logger that combines previous loggers
        auto csv_logger = std::make_shared<sdm::CSVLogger>(this->getName(), list_logs);

        // Build a multi logger that combines previous loggers
        logger_ = std::make_shared<sdm::MultiLogger>(std::vector<std::shared_ptr<Logger>>{std_logger, file_logger, csv_logger});
    }

    void SARSA::logging()
    {

        if (auto derived = std::dynamic_pointer_cast<BeliefMDPInterface>(getEnv()))
        {
            if (auto pwlc = sdm::isInstanceOf<PWLCQValueFunction>(this->q_value_))
            {

                std::ostringstream output_nstates;
                output_nstates << "[ ";

                for (const auto &pwlc_q_t : pwlc->representation)
                {
                    output_nstates << "\033[1m" << config::SDMS_THEME_1 << pwlc_q_t.size() << "\033[0m(";
                    unsigned long size_support = 0;
                    for (const auto &pwlc_q_t_tuple : pwlc_q_t)
                    {
                        size_support += pwlc_q_t_tuple.second->getState().size();
                    }
                    output_nstates << size_support << ")  ";
                }
                output_nstates << "]";

                // Print in loggers some execution variables
                this->logger_->log(this->episode,
                                   this->global_step,
                                   this->q_value_->getValueAt(getEnv()->reset(), 0),
                                   (this->current > -100000) ? this->current : -10000,
                                   getExecutionTime(),
                                   this->exploration_process->getEpsilon(),
                                   this->learning_rate,
                                   output_nstates.str());
            }
            else
            {
                // Print in loggers some execution variables
                this->logger_->log(this->episode,
                                   this->global_step,
                                   this->q_value_->getValueAt(getEnv()->reset(), 0),
                                   (this->current > -100000) ? this->current : -10000,
                                   getExecutionTime(),
                                   this->exploration_process->getEpsilon(),
                                   this->learning_rate,
                                   derived->getMDPGraph()->getNumNodes());
            }
        }
        else
        {
            // Print in loggers some execution variables
            this->logger_->log(this->episode,
                               this->global_step,
                               this->q_value_->getValueAt(getEnv()->reset(), 0),
                               (this->current > -100000) ? this->current : -10000,
                               getExecutionTime(),
                               this->exploration_process->getEpsilon(),
                               this->learning_rate);
        }
    }
}
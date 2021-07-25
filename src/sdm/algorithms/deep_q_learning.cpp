#include <sdm/algorithms/deep_q_learning.hpp>
#include <sdm/world/occupancy_mdp.hpp>

namespace sdm
{

    DeepQLearning::DeepQLearning(std::shared_ptr<GymInterface> &env,
                         std::shared_ptr<ExperienceMemoryInterface> experience_memory,
                         std::shared_ptr<DQN> policy_net,
                         std::shared_ptr<DQN> target_net,
                         std::shared_ptr<QValueBackupInterface> backup, /////
                         std::shared_ptr<EpsGreedy> exploration,
                         number horizon,
                         double discount,
                         double lr,
                         unsigned long num_episodes,
                         std::string name
                         ) : env_(env),
                             experience_memory_(experience_memory),
                             policy_net_(policy_net),
                             target_net_(target_net),
                             backup_(backup),
                             exploration_process(exploration),
                             horizon_(horizon),
                             discount_(discount),
                             lr_(lr),
                             num_episodes_(num_episodes),
                             target_update_(10), /////
                             name_(name)
    {
    }

    void DeepQLearning::initLogger()
    {
        std::string format = "#> Episode : {}\tEpsilon : {}\tValue : {}\tQ Value Error : {}\tT(s) : {}\n";

        auto std_logger = std::make_shared<sdm::StdLogger>(format);
        auto file_logger = std::make_shared<sdm::FileLogger>(this->name_ + ".txt", format);
        auto csv_logger = std::make_shared<sdm::CSVLogger>(this->name_, std::vector<std::string>{"Episode", "Epsilon", "Value", "Q Value Error", "Time"});

        this->logger_ = std::make_shared<sdm::MultiLogger>(std::vector<std::shared_ptr<Logger>>{std_logger, file_logger, csv_logger});
    }

    void DeepQLearning::do_initialize()
    {
        this->initLogger();
        std::cout << "horizon_ " << horizon_ << std::endl;
        std::cout << "discount_ " << discount_ << std::endl;
        std::cout << "lr_ " << lr_ << std::endl;
        std::cout << "num_episodes_ " << num_episodes_ << std::endl;
    }

    void DeepQLearning::do_solve()
    {
        this->episode = 0;
        this->t_begin = clock();

        this->exploration_process->reset(this->num_episodes_);

        while (this->episode < this->num_episodes_)
        {
            // Update exploration process
            this->exploration_process->update(this->episode);

            // Do one episode
            this->do_episode();

            // Test current policy and write logs
            if (this->episode % this->log_freq == 0)
            {
                this->logger_->log(
                    this->episode, 
                    this->exploration_process->getEpsilon(), 
                    this->do_evaluate(), 
                    this->q_value_error,
                    (float)(clock() - this->t_begin) / CLOCKS_PER_SEC
                );
            }

            if (this->episode % this->target_update_freq == 0)
            {
                this->update_target();
            }

            if (this->episode % this->save_freq == 0)
            {
                this->do_save();
            }


        }

    }

    void DeepQLearning::do_save()
    {
        /////
    }

    void DeepQLearning::do_test()
    {

    }

    double DeepQLearning::do_evaluate()
    {
        double total_reward = 0;
        for (number episod = 0; episod < this->test_n; episod++)
        {
            number ste = 0;
            this->observation = this->env_->reset();
            this->action = this->backup_->getGreedyAction(this->observation->toState(), ste);
            // this->action = this->env_->getRandomAction(this->observation, ste);
            for (number ste = 0; ste < this->horizon_; ste++)
            {
                std::tie(this->next_observation, this->rewards_, this->is_done) = this->env_->step(this->action);
                total_reward += pow(this->discount_, ste) * this->rewards_[1];
                this->next_action = this->backup_->getGreedyAction(this->next_observation->toState(), ste + 1);
                // this->next_action = this->env_->getRandomAction(this->next_observation, ste + 1);
                this->observation = next_observation;
                this->action = next_action;
            }
        }
        return total_reward / this->test_n;
    }

    void DeepQLearning::do_episode()
    {
        this->step = 0;
        this->q_value_error = 0;
        this->observation = this->env_->reset();
        this->action = this->select_action(this->observation, this->step);

        while (this->step < this->horizon_)
        {
            this->do_step();
        }

        // Backup and get Q Value Error
        this->q_value_error += this->backup_->update();

        this->episode++;
    }

    void DeepQLearning::do_step()
    {   
        // std::cout << "DeepQLearning::do_step() a " << this->step << std::endl;

        // One step in env and get next observation and rewards
        std::tie(this->next_observation, this->rewards_, this->is_done) = this->env_->step(this->action);

        // std::cout << "DeepQLearning::do_step() b " << this->step << std::endl;

        this->next_action = this->select_action(this->next_observation, this->step + 1);

        // std::cout << "DeepQLearning::do_step() c " << this->step << std::endl;

        // Push experience to memory
        this->experience_memory_->push(this->observation, this->action, this->rewards_[0], this->next_observation, this->next_action, this->step);

        this->observation = this->next_observation;
        this->action = this->next_action;
        this->step++;
    }


    void DeepQLearning::update_target()
    {
        // Create std::stringstream stream.
		std::stringstream stream;
		// Save the parameters of agent 1's policy net into the stream.
		torch::save(*this->policy_net_, stream);
		// Load those weights from the stream into agent 1's target net.
		torch::load(*this->target_net_, stream);

    }

    std::shared_ptr<Action> DeepQLearning::select_action(const std::shared_ptr<Observation> &observation, number t)
    {
        if ((rand() / double(RAND_MAX)) < this->exploration_process->getEpsilon())
        {
            // std::cout << "-------- RANDOM ---------" << std::endl;
            return this->env_->getRandomAction(observation, t);
        }
        else
        {
            // std::cout << "-------- GREEDY ---------" << std::endl;
            return this->backup_->getGreedyAction(observation->toState(), t);
            // return this->env_->getRandomAction(observation, t);
        }
    }

} // namespace sdm

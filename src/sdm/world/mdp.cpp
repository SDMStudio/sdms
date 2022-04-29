#include <sdm/world/mdp.hpp>
#include <sdm/core/space/discrete_space.hpp>
#include <regex>

namespace sdm
{
    MDP::MDP() {}

    MDP::MDP(const std::shared_ptr<StateSpace> &state_space,
             const std::shared_ptr<ActionSpace> &action_space,
             const std::shared_ptr<RewardModel> &reward_space,
             const std::shared_ptr<StateDynamicsInterface> &state_dynamics,
             const std::shared_ptr<Distribution<std::shared_ptr<State>>> &start_distribution,
             number horizon,
             double discount,
             Criterion criterion) : num_agents_(1),
                                    horizon_(horizon),
                                    discount_(discount),
                                    criterion_(criterion),
                                    state_space_(state_space),
                                    action_space_(action_space),
                                    reward_space_(reward_space),
                                    state_dynamics_(state_dynamics),
                                    start_distribution_(start_distribution)
    {
    }

    MDP::~MDP() {}

    void MDP::configure(Config config)
    {
        double discount = config.get<double>("discount", -1.);
        if (discount >= 0)
            this->setDiscount(discount);

        int horizon = config.get("horizon", -1);
        if (horizon >= 0)
            this->setHorizon(horizon);
    }

    number MDP::getNumAgents() const
    {
        return this->num_agents_;
    }

    double MDP::getDiscount(number) const
    {
        return this->discount_;
    }

    double MDP::getWeightedDiscount(number t) const
    {
        return pow(this->getDiscount(t), t);
    }

    void MDP::setDiscount(double discount)
    {
        this->discount_ = discount;
    }

    number MDP::getHorizon() const
    {
        return this->horizon_;
    }

    void MDP::setHorizon(number horizon)
    {
        this->horizon_ = horizon;
    }

    std::shared_ptr<Distribution<std::shared_ptr<State>>> MDP::getStartDistribution() const
    {
        return this->start_distribution_;
    }

    std::set<std::shared_ptr<State>> MDP::getReachableStates(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t) const
    {
        return this->state_dynamics_->getReachableStates(state, action, t);
    }
    std::shared_ptr<StateDynamicsInterface> MDP::getStateDynamics() const
    {
        return this->state_dynamics_;
    }

    std::shared_ptr<RewardModel> MDP::getRewardSpace() const
    {
        return this->reward_space_;
    }

    double MDP::getReward(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t) const
    {
        return this->reward_space_->getReward(state, action, 0, t);
    }

    double MDP::getMinReward(number t) const
    {
        return this->reward_space_->getMinReward(0, t);
    }

    double MDP::getMaxReward(number t) const
    {
        return this->reward_space_->getMaxReward(0, t);
    }

    double MDP::getTransitionProbability(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, number t) const
    {
        return this->state_dynamics_->getTransitionProbability(state, action, next_state, t);
    }

    std::shared_ptr<ActionSpace> MDP::getActionSpace(number) const
    {
        return this->action_space_;
    }

    std::shared_ptr<StateSpace> MDP::getStateSpace(number) const
    {
        return this->state_space_;
    }

    std::shared_ptr<State> MDP::reset()
    {
        this->current_timestep_ = 0;
        this->setInternalState(this->getStartDistribution()->sample());
        return this->getInternalState();
    }

    void MDP::setInternalState(std::shared_ptr<State> state)
    {
        this->internal_state_ = state;
    }

    std::shared_ptr<State> MDP::getInternalState() const
    {
        return this->internal_state_;
    }

    std::shared_ptr<Observation> MDP::sampleNextObservation(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t)
    {
        std::shared_ptr<State> next_state = this->state_dynamics_->getNextStateDistribution(state, action, t)->sample();
        this->setInternalState(next_state);
        return this->getInternalState();
    }

    std::tuple<std::shared_ptr<State>, std::vector<double>, bool> MDP::step(std::shared_ptr<Action> action)
    {
        double reward = this->getReward(this->getInternalState(), action);

        auto observation = this->sampleNextObservation(this->getInternalState(), action, this->current_timestep_)->toState();

        this->current_timestep_++;

        bool is_done = (this->getHorizon() > 0) ? (this->getHorizon() <= this->current_timestep_) : (1000 <= this->current_timestep_);

        return std::make_tuple(observation, std::vector<double>{reward}, is_done);
    }

    std::tuple<std::shared_ptr<State>, std::vector<double>, bool> MDP::step(std::shared_ptr<Action> action, bool increment_timestep)
    {
        double reward = this->getReward(this->getInternalState(), action);

        auto observation = this->sampleNextObservation(this->getInternalState(), action, this->current_timestep_);

        if (increment_timestep)
            this->current_timestep_++;

        bool is_done = (this->getHorizon() > 0) ? (this->getHorizon() <= this->current_timestep_) : (1000 <= this->current_timestep_);

        return std::make_tuple(observation->toState(), std::vector<double>{reward}, is_done);
    }

    // Useless
    std::shared_ptr<ActionSpace> MDP::getActionSpaceAt(const std::shared_ptr<State> &, number t)
    {
        return this->getActionSpace(t);
    }

    std::shared_ptr<Action> MDP::getRandomAction(const std::shared_ptr<State> &, number t)
    {
        return this->getActionSpace(t)->sample();
    }

    std::string MDP::toStdFormat()
    {
        if (this->getStateSpace()->isDiscrete() && this->getActionSpace()->isDiscrete())
        {

            auto state_space = std::static_pointer_cast<DiscreteStateSpace>(this->getStateSpace());
            auto action_space = std::static_pointer_cast<DiscreteActionSpace>(this->getActionSpace());

            std::ostringstream res;
            number n_agents = 1;

            res << "agents: " << n_agents << std::endl;
            res << "discount: " << this->getDiscount() / 1.0 << std::endl;
            res << "values: \"" << ((this->criterion_ == Criterion::COST_MIN) ? "cost" : "reward") << "\"" << std::endl;
            res << "states: " << state_space->getNumItems() << std::endl;
            res << "start: \"uniform\"" << std::endl;

            res << "actions: \n";
            res << action_space->getNumItems() << "\n";

            auto state_end_iter = state_space->end();
            for (auto state_iter = state_space->begin(); !state_iter->equal(state_end_iter); state_iter = state_iter->next())
            {
                auto state = state_iter->getCurrent();
                auto action_end_iter = action_space->end();
                for (auto action_iter = action_space->begin(); !action_iter->equal(action_end_iter); action_iter = action_iter->next())
                {
                    auto action = action_iter->getCurrent();
                    auto next_state_end_iter = state_space->end();
                    for (auto next_state_iter = state_space->begin(); !next_state_iter->equal(next_state_end_iter); next_state_iter = next_state_iter->next())
                    {
                        auto next_state = next_state_iter->getCurrent();
                        res << "T: " << action_space->getItemIndex(action)
                            << " : " << state_space->getItemIndex(state)
                            << " : " << state_space->getItemIndex(next_state)
                            << " : " << this->getTransitionProbability(state, action, next_state->toState())
                            << std::endl;
                    }
                }
            }

            state_end_iter = state_space->end();
            for (auto state_iter = state_space->begin(); !state_iter->equal(state_end_iter); state_iter = state_iter->next())
            {
                auto state = state_iter->getCurrent();
                auto action_end_iter = action_space->end();
                for (auto action_iter = action_space->begin(); !action_iter->equal(action_end_iter); action_iter = action_iter->next())
                {
                    auto action = action_iter->getCurrent();

                    res << "R: " << action_space->getItemIndex(action)
                        << " : " << state_space->getItemIndex(state)
                        << " : " << this->getReward(state, action)
                        << std::endl;
                }
            }
            return res.str();
        }
        else
        {
            return "No known Standard format for Continuous MDPs.";
        }
    }

    std::string MDP::toXML()
    {
        if (this->getStateSpace()->isDiscrete() && this->getActionSpace()->isDiscrete())
        {
            return "No known XML format for Problems.";
        }
        else
        {
            return "No known XML format for Continuous MDPs.";
        }
    }

    std::string MDP::toJSON()
    {
        std::cout << "toJSON : Not implemented method" << std::endl;
        return "Not implemented method";
    }

    void MDP::generateFile(std::string filename)
    {
        std::ofstream myfile;
        if (regex_match(filename, std::regex(".*\\.json$")) || regex_match(filename, std::regex(".*\\.JSON$")))
        {
            myfile = std::ofstream(filename);
            myfile << this->toJSON();
        }
        else if (regex_match(filename, std::regex(".*\\.xml$")) || regex_match(filename, std::regex(".*\\.XML$")))
        {
            myfile = std::ofstream(filename);
            myfile << this->toXML();
        }
        else
        {
            myfile = std::ofstream(filename);
            myfile << this->toStdFormat();
        }
        myfile.close();
    }
}
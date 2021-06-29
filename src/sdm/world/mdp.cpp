#include <sdm/world/mdp.hpp>
#include <sdm/core/space/discrete_space.hpp>
#include <regex>

namespace sdm
{

    MDP::MDP(const std::shared_ptr<Space> &state_space,
             const std::shared_ptr<Space> &action_space,
             const std::shared_ptr<RewardInterface> &reward,
             const std::shared_ptr<StateDynamicsInterface> &state_dynamics,
             const std::shared_ptr<Distribution<std::shared_ptr<State>>> &start_distrib,
             number horizon,
             double discount,
             Criterion criterion) : num_agents_(1),
                                    horizon_(horizon),
                                    discount_(discount),
                                    criterion_(criterion),
                                    state_space_(state_space),
                                    action_space_(action_space),
                                    reward_(reward),
                                    state_dynamics_(state_dynamics),
                                    start_distrib_(start_distrib)
    {
    }

    MDP::~MDP() {}

    number MDP::getNumAgents() const
    {
        return this->num_agents_;
    }

    double MDP::getDiscount(number) const
    {
        return this->discount_;
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
        return this->start_distrib_;
    }

    std::set<std::shared_ptr<State>> MDP::getReachableStates(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t) const
    {
        return this->state_dynamics_->getReachableStates(state, action, t);
    }
    std::shared_ptr<StateDynamicsInterface> MDP::getStateDynamics() const
    {
        return this->state_dynamics_;
    }

    std::shared_ptr<RewardInterface> MDP::getReward() const
    {
        return this->reward_;
    }

    double MDP::getReward(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t) const
    {
        return this->reward_->getReward(state, action, t);
    }

    double MDP::getMinReward(number t) const
    {
        return this->reward_->getMinReward(t);
    }

    double MDP::getMaxReward(number t) const
    {
        return this->reward_->getMaxReward(t);
    }

    double MDP::getTransitionProbability(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, number t) const
    {
        return this->state_dynamics_->getTransitionProbability(state, action, next_state, t);
    }

    std::shared_ptr<Space> MDP::getActionSpace(number) const
    {
        return this->action_space_;
    }

    std::shared_ptr<Space> MDP::getStateSpace(number) const
    {
        return this->state_space_;
    }

    std::shared_ptr<Observation> MDP::reset()
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

    std::shared_ptr<Observation> MDP::sampleNextObservation(const std::shared_ptr<State>& state, const std::shared_ptr<Action>& action, number t) 
    {
        auto next_state_distribution = this->state_dynamics_->getNextStateDistribution(state, action, t);
        auto next_state = next_state_distribution->sample();
        this->setInternalState(next_state);
        return this->getInternalState();
    }

    std::tuple<std::shared_ptr<Observation>, std::vector<double>, bool> MDP::step(std::shared_ptr<Action> action)
    {
        auto reward = this->getReward(this->getInternalState(), action);

        auto observation = this->sampleNextObservation(this->getInternalState(), action, this->current_timestep_);

        bool is_done = (this->getHorizon() > 0) ? (this->getHorizon() <= this->current_timestep_) : (1000 <= this->current_timestep_);
        
        this->current_timestep_++;

        return std::make_tuple(observation, std::vector<double>{reward}, is_done);
    }

    std::shared_ptr<Space> MDP::getActionSpaceAt(const std::shared_ptr<Observation> &, number t)
    {
        return this->getActionSpace(t);
    }

    std::shared_ptr<Space> MDP::getActionSpaceAt(number t)
    {
        return this->getActionSpace(t);
    }

    std::string MDP::toStdFormat()
    {
        if (this->getStateSpace()->isDiscrete() && this->getActionSpace()->isDiscrete())
        {

            auto state_space = std::static_pointer_cast<DiscreteSpace>(this->getStateSpace());
            auto action_space = std::static_pointer_cast<DiscreteSpace>(this->getActionSpace());

            std::ostringstream res;
            number n_agents = 1;

            res << "agents: " << n_agents << std::endl;
            res << "discount: " << this->getDiscount() / 1.0 << std::endl;
            res << "values: \"" << ((this->criterion_ == Criterion::COST_MIN) ? "cost" : "reward") << "\"" << std::endl;
            res << "states: " << state_space->getNumItems() << std::endl;
            res << "start: \"uniform\"" << std::endl;

            res << "actions: \n";
            res << action_space->getNumItems() << "\n";

            for (const auto &state : *state_space)
            {
                for (const auto &action : *action_space)
                {
                    for (const auto &next_state : *state_space)
                    {
                        res << "T: " << action_space->getItemIndex(action)
                            << " : " << state_space->getItemIndex(state)
                            << " : " << state_space->getItemIndex(next_state)
                            << " : " << this->getTransitionProbability(state->toState(), action->toAction(), next_state->toState())
                            << std::endl;
                    }
                }
            }

            for (const auto &state : *state_space)
            {
                for (const auto &action : *action_space)
                {
                    res << "R: " << action_space->getItemIndex(action)
                        << " : " << state_space->getItemIndex(state)
                        << " : " << this->getReward(state->toState(), action->toAction())
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
            // std::ostringstream res;
            // number n_agents = this->getNumAgents();

            // number ag;
            // res << "<DiscreteMDP>" << std::endl;
            // res << "\t<preamble>" << std::endl;
            // // res << "\t\t<soundness>" << this->isSound() << "</soundness>" << std::endl;
            // res << "\t\t<agents>" << n_agents << "</agents>" << std::endl;
            // res << "\t\t<discount>" << this->getDiscount() << "</discount>" << std::endl;
            // res << "\t\t<states>" << std::static_pointer_cast<DiscreteSpace>(this->getStateSpace())->getNumItems() << "</states>" << std::endl;
            // res << "\t\t<start> [";
            // for (const auto &state : *this->getStateSpace())
            // {
            //     res << this->getStartDistribution()->getProbability(std::static_pointer_cast<State>(state), nullptr) << " ";
            // }
            // res << "] </start>" << std::endl;

            // res << "\t\t<actions>" << std::endl;
            // for (ag = 0; ag < n_agents; ++ag)
            //     res << "\t\t\t<agent id=\"" << ag << "\">" << this->getActionSpace()->getSpace(ag)->getNumItems() << "</agent>" << std::endl;
            // res << "\t\t</actions>" << std::endl;

            // res << "\t\t<observations>" << std::endl;
            // for (ag = 0; ag < n_agents; ++ag)
            //     res << "\t\t\t<agent id=\"" << ag << "\">" << this->getObsSpace()->getSpace(ag)->getNumItems() << "</agent>" << std::endl;
            // res << "\t\t</observations>" << std::endl;

            // res << "\t</preamble>" << std::endl;
            // res << "\t<param>" << std::endl;

            // number ja;
            // res << "\t\t<reward>" << std::endl;
            // for (ja = 0; ja < this->getActionSpace()->getNumItems(); ++ja)
            // {
            //     std::vector<number> v_ja = this->getActionSpace()->single2joint(ja);
            //     res << "\t\t\t<reward-entry joint-action=\"";
            //     for (number ag = 0; ag < n_agents; ++ag)
            //     {
            //         res << v_ja[ag] << " ";
            //     }
            //     res << "\" >" << std::endl;
            //     res << "\t\t\t\t" << this->getReward()->getReward(ja) << std::endl;
            //     res << "\t\t\t</reward-entry>" << std::endl;
            // }

            // res << "\t\t</reward>" << std::endl;

            // observation jz;
            // res << "\t\t<dynamics>" << std::endl;
            // for (ja = 0; ja < this->getActionSpace()->getNumItems(); ++ja)
            //     for (jz = 0; jz < this->getObsSpace()->getNumItems(); ++jz)
            //     {
            //         res << "\t\t\t<dynamics-entry jaction=\"" << ja << "\" jobservation=\"" << jz << "\">" << std::endl;
            //         res << "\t\t\t\t" << this->getObsDynamics()->getDynamics(ja, jz) << std::endl;
            //         res << "\t\t\t</dynamics-entry>" << std::endl;
            //     }
            // res << "\t\t</dynamics>" << std::endl;

            // res << "\t</param>" << std::endl;
            // res << "</decpomdp>" << std::endl;

            // return res.str();
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
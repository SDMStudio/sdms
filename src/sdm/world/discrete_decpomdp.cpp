#include <sdm/world/discrete_decpomdp.hpp>
#include <sdm/parser/parser.hpp>
#include <regex>

namespace sdm
{

    DiscreteDecPOMDP::DiscreteDecPOMDP()
    {
    }

    DiscreteDecPOMDP::DiscreteDecPOMDP(std::string &filename)
    {
        *this = *(parser::parse_file(filename.c_str()));
    }

    DiscreteDecPOMDP::DiscreteDecPOMDP(DiscreteDecPOMDP &copy)
        : StochasticProcessBase<DiscreteSpace<number>, std::discrete_distribution<number>>(copy.getStateSpace(), copy.getStartDistrib()),
          PartiallyObservableDecisionProcess<DiscreteSpace<number>,
                                             MultiDiscreteSpace<number>,
                                             MultiDiscreteSpace<number>,
                                             StateDynamics,
                                             ObservationDynamics,
                                             Reward,
                                             std::discrete_distribution<number>>(copy)
    {
    }

    DiscreteDecPOMDP::DiscreteDecPOMDP(std::shared_ptr<DiscreteSpace<number>> state_sp, std::shared_ptr<MultiDiscreteSpace<number>> action_sp, std::shared_ptr<MultiDiscreteSpace<number>> obs_sp, std::shared_ptr<StateDynamics> state_dyn, std::shared_ptr<ObservationDynamics> obs_dyn, std::shared_ptr<Reward> rew_f, std::discrete_distribution<number> start_distrib, number planning_horizon, double discount, Criterion criterion)
        : StochasticProcessBase<DiscreteSpace<number>, std::discrete_distribution<number>>(state_sp, start_distrib),
          PartiallyObservableDecisionProcess<DiscreteSpace<number>,
                                             MultiDiscreteSpace<number>,
                                             MultiDiscreteSpace<number>,
                                             StateDynamics,
                                             ObservationDynamics,
                                             Reward,
                                             std::discrete_distribution<number>>(state_sp,
                                                                                 action_sp,
                                                                                 obs_sp,
                                                                                 state_dyn,
                                                                                 obs_dyn,
                                                                                 rew_f,
                                                                                 start_distrib,
                                                                                 planning_horizon,
                                                                                 discount,
                                                                                 criterion)

    {
    }

    // // Other methods
    std::shared_ptr<DiscretePOMDP> DiscreteDecPOMDP::toPOMDP()
    {
        std::shared_ptr<DiscreteSpace<number>> new_action_space = std::make_shared<DiscreteSpace<number>>(this->getActionSpace()->getNumJointItems());
        std::shared_ptr<DiscreteSpace<number>> new_obs_space = std::make_shared<DiscreteSpace<number>>(this->getObsSpace()->getNumJointItems());
        return std::make_shared<DiscretePOMDP>(this->getStateSpace(), new_action_space, new_obs_space, this->getStateDynamics(), this->getObsDynamics(), this->getReward(), this->getStartDistrib(), this->getPlanningHorizon(), this->getDiscount(), this->getCriterion());
    }

    std::shared_ptr<DiscreteMMDP> DiscreteDecPOMDP::toMMDP()
    {
        return std::make_shared<DiscreteMMDP>(this->getStateSpace(), this->getActionSpace(), this->getStateDynamics(), this->getReward(), this->getStartDistrib(), this->getPlanningHorizon(), this->getDiscount(), this->getCriterion());
    }

    // std::shared_ptr<BeliefMDP> DiscreteDecPOMDP::toBeliefMDP()
    // {
    // }

    std::string DiscreteDecPOMDP::toStdFormat()
    {

        std::ostringstream res;
        number n_agents = this->getNumAgents();

        res << "agents: " << n_agents << std::endl;
        res << "discount: " << this->getDiscount() / 1.0 << std::endl;
        res << "values: \"" << ((this->getCriterion() == Criterion::COST_MIN) ? "cost" : "reward") << "\"" << std::endl;
        res << "states: " << this->getStateSpace()->getNumItems() << std::endl;
        res << "start: \"uniform\"" << std::endl;
        res << "actions: \n";

        for (number ag = 0; ag < n_agents; ag++)
        {
            res << this->getActionSpace()->getSpace(ag)->getNumItems() << "\n";
        }
        res << "observations: \n";
        for (number ag = 0; ag < n_agents; ag++)
        {
            res << this->getObsSpace()->getSpace(ag)->getNumItems() << "\n";
        }

        for (number x = 0; x < this->getStateSpace()->getNumItems(); ++x)
        {
            for (number u = 0; u < this->getActionSpace()->getNumItems(); u++)
            {
                std::vector<number> ja = this->getActionSpace()->single2joint(u);
                for (number y = 0; y < this->getStateSpace()->getNumItems(); ++y)
                {
                    res << "T: ";
                    for (number agent = 0; agent < n_agents; ++agent)
                    {
                        res << ja[agent] << " ";
                    }
                    res << ": " << x << " : " << y << " : " << this->getStateDynamics()->getTransitionProbability(x, u, y) << std::endl;
                }
            }
        }

        for (number y = 0; y < this->getStateSpace()->getNumItems(); ++y)
        {
            for (number u = 0; u < this->getActionSpace()->getNumItems(); u++)
            {
                std::vector<number> ja = this->getActionSpace()->single2joint(u);
                for (number z = 0; z < this->getObsSpace()->getNumItems(); ++z)
                {
                    res << "O: ";
                    for (number agent = 0; agent < n_agents; ++agent)
                    {
                        res << ja[agent] << " ";
                    }
                    res << " : " << y << " : ";
                    std::vector<number> jz = this->getObsSpace()->single2joint(z);
                    for (number agent = 0; agent < n_agents; ++agent)
                    {
                        res << jz[agent] << " ";
                    }
                    res << ": " << this->getObsDynamics()->getObservationProbability(u, z, y) << std::endl;
                }
            }
        }

        for (number x = 0; x < this->getStateSpace()->getNumItems(); ++x)
        {
            for (number u = 0; u < this->getActionSpace()->getNumItems(); u++)
            {
                res << "R: ";
                std::vector<number> ja = this->getActionSpace()->single2joint(u);
                for (number agent = 0; agent < n_agents; ++agent)
                {
                    res << ja[agent] << " ";
                }
                res << ": " << x << " :";
                res << " " << this->getReward()->getReward(x, u);
                res << std::endl;
            }
        }
        return res.str();
    }

    std::string DiscreteDecPOMDP::toXML()
    {
        std::ostringstream res;
        number n_agents = this->getNumAgents();

        number ag;
        res << "<decpomdp>" << std::endl;
        res << "\t<preamble>" << std::endl;
        // res << "\t\t<soundness>" << this->isSound() << "</soundness>" << std::endl;
        res << "\t\t<agents>" << n_agents << "</agents>" << std::endl;
        res << "\t\t<discount>" << this->getDiscount() << "</discount>" << std::endl;
        res << "\t\t<states>" << this->getStateSpace()->getNumItems() << "</states>" << std::endl;
        res << "\t\t<start> [";
        for (auto v : this->getStartDistrib().probabilities())
        {
            res << v << " ";
        }
        res << "] </start>" << std::endl;

        res << "\t\t<actions>" << std::endl;
        for (ag = 0; ag < n_agents; ++ag)
            res << "\t\t\t<agent id=\"" << ag << "\">" << this->getActionSpace()->getSpace(ag)->getNumItems() << "</agent>" << std::endl;
        res << "\t\t</actions>" << std::endl;

        res << "\t\t<observations>" << std::endl;
        for (ag = 0; ag < n_agents; ++ag)
            res << "\t\t\t<agent id=\"" << ag << "\">" << this->getObsSpace()->getSpace(ag)->getNumItems() << "</agent>" << std::endl;
        res << "\t\t</observations>" << std::endl;

        res << "\t</preamble>" << std::endl;
        res << "\t<param>" << std::endl;

        number ja;
        res << "\t\t<reward>" << std::endl;
        for (ja = 0; ja < this->getActionSpace()->getNumItems(); ++ja)
        {
            std::vector<number> v_ja = this->getActionSpace()->single2joint(ja);
            res << "\t\t\t<reward-entry joint-action=\"";
            for (number ag = 0; ag < n_agents; ++ag)
            {
                res << v_ja[ag] << " ";
            }
            res << "\" >" << std::endl;
            res << "\t\t\t\t" << this->getReward()->getReward(ja) << std::endl;
            res << "\t\t\t</reward-entry>" << std::endl;
        }

        res << "\t\t</reward>" << std::endl;

        observation jz;
        res << "\t\t<dynamics>" << std::endl;
        for (ja = 0; ja < this->getActionSpace()->getNumItems(); ++ja)
            for (jz = 0; jz < this->getObsSpace()->getNumItems(); ++jz)
            {
                res << "\t\t\t<dynamics-entry jaction=\"" << ja << "\" jobservation=\"" << jz << "\">" << std::endl;
                res << "\t\t\t\t" << this->getObsDynamics()->getDynamics(ja, jz) << std::endl;
                res << "\t\t\t</dynamics-entry>" << std::endl;
            }
        res << "\t\t</dynamics>" << std::endl;

        res << "\t</param>" << std::endl;
        res << "</decpomdp>" << std::endl;

        return res.str();
    }

    std::string DiscreteDecPOMDP::toJSON()
    {
        std::cout << "toJSON : Not implemented method" << std::endl;
        return "Not implemented method";
    }

    void DiscreteDecPOMDP::generateFile(std::string filename)
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
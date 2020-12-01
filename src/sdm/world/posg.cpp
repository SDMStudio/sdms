#include <sdm/world/posg.hpp>
#include <regex>

namespace sdm
{

    POSG::POSG() {}

    POSG::POSG(const DecisionProcess &stochastic_game) : DecisionProcess(stochastic_game.getStateSpace(), stochastic_game.getAgentSpace(), stochastic_game.getActionSpace(), stochastic_game.getStartDistrib())
    {
    }

    // POSG::POSG(const std::string &filename) {}

    // POSG::POSG(number state_sp, number action_sp, const std::vector<double> & start_distrib) : SG(state_sp, action_sp, start_distrib) {}

    POSG::POSG(const DiscreteSpace &state_sp, const DiscreteSpace &agent_sp)
    {
        this->state_space_ = state_sp;
        this->agent_space_ = agent_sp;
    }

    POSG::POSG(const DiscreteSpace &state_sp, const DiscreteSpace &agent_sp, const MultiDiscreteSpace &action_sp, const MultiDiscreteSpace &obs_sp,
               const StateDynamics &s_dyn, const ObservationDynamics &o_dyn, const std::vector<Reward> &rews, const Vector &start_distrib)
        : o_dynamics_(o_dyn)
    {
        this->state_space_ = state_sp;
        this->setStartDistrib(start_distrib);
        this->agent_space_ = agent_sp;
        this->action_space_ = action_sp;
        this->s_dynamics_ = s_dyn;
        this->rew_ = rews;
        this->obs_spaces_ = obs_sp;
    }

    const ObservationDynamics &POSG::getObsDynamics() const
    {
        return this->o_dynamics_;
    }

    double POSG::getObservationProbability(number jaction, number jobservation, number state) const
    {
        return this->o_dynamics_.getObservationProbability(jaction, jobservation, state);
    }

    double POSG::getObservationProbability(std::vector<number> jaction, std::vector<number> jobservation, number state) const
    {
        return this->getObservationProbability(this->action_space_.joint2single(jaction), this->obs_spaces_.joint2single(jobservation), state);
    }

    const Matrix &POSG::getObservations(number jaction) const
    {
        return this->o_dynamics_.getObservations(jaction);
    }

    const Matrix &POSG::getObservations(std::vector<number> jaction) const
    {
        return this->getObservations(this->action_space_.joint2single(jaction));
    }

    double POSG::getDynamics(number cstate, number jaction, number jobservation, number nstate) const
    {
        return this->o_dynamics_.getDynamics(cstate, jaction, jobservation, nstate);
    }

    const Matrix &POSG::getDynamics(number jaction, number jobservation) const
    {
        return this->o_dynamics_.getDynamics(jaction, jobservation);
    }

    std::string POSG::toStdFormat() const
    {
        std::ostringstream res;

        res << "agents: " << this->getNumAgents() << std::endl;
        res << "discount: " << this->getDiscount() / 1.0 << std::endl;
        res << "values: \"reward\"" << std::endl;
        res << "states: " << this->getNumStates() << std::endl;
        res << "start: \"uniform\"" << std::endl;
        res << "actions: \n";
        for (number ag = 0; ag < this->getNumAgents(); ag++)
        {
            res << this->getActionSpace().getNumElements(ag) << "\n";
        }
        res << "observations: \n";
        for (number ag = 0; ag < this->getNumAgents(); ag++)
        {
            res << this->getObsSpace().getNumElements(ag) << "\n";
        }

        for (number x = 0; x < this->getNumStates(); ++x)
        {
            for (number u = 0; u < this->getNumJActions(); u++)
            {
                std::vector<number> ja = this->getActionSpace().getJointElement(u);
                for (number y = 0; y < this->getNumStates(); ++y)
                {
                    res << "T: ";
                    for (number agent = 0; agent < this->getNumAgents(); ++agent)
                    {
                        res << ja[agent] << " ";
                    }
                    res << ": " << x << " : " << y << " : " << this->getStateDynamics().getTransitionProbability(x, u, y) << std::endl;
                }
            }
        }

        for (number y = 0; y < this->getNumStates(); ++y)
        {
            for (number u = 0; u < this->getNumJActions(); u++)
            {
                std::vector<number> ja = this->getActionSpace().getJointElement(u);
                for (number z = 0; z < this->getNumJObservations(); ++z)
                {
                    res << "O: ";
                    for (number agent = 0; agent < this->getNumAgents(); ++agent)
                    {
                        res << ja[agent] << " ";
                    }
                    res << " : " << y << " : ";
                    std::vector<number> jz = this->getObsSpace().getJointElement(z);
                    for (number agent = 0; agent < this->getNumAgents(); ++agent)
                    {
                        res << jz[agent] << " ";
                    }
                    res << ": " << this->getObsDynamics().getObservationProbability(u, z, y) << std::endl;
                }
            }
        }

        for (number x = 0; x < this->getNumStates(); ++x)
        {
            for (number u = 0; u < this->getNumJActions(); u++)
            {
                res << "R: ";
                std::vector<number> ja = this->getActionSpace().getJointElement(u);
                for (number agent = 0; agent < this->getNumAgents(); ++agent)
                {
                    res << ja[agent] << " ";
                }
                res << ": " << x << " :";
                for (Reward r : this->rew_)
                {
                    res << " " << r.getReward(x, u);
                }
                res << std::endl;
            }
        }
        return res.str();
    }

    std::string POSG::toXML() const
    {
        std::ostringstream res;

        number ag;
        res << "<decpomdp>" << std::endl;
        res << "\t<preamble>" << std::endl;
        // res << "\t\t<soundness>" << this->isSound() << "</soundness>" << std::endl;
        res << "\t\t<agents>" << this->getNumAgents() << "</agents>" << std::endl;
        res << "\t\t<discount>" << this->getDiscount() << "</discount>" << std::endl;
        res << "\t\t<states>" << this->getNumStates() << "</states>" << std::endl;
        res << "\t\t<start>" << this->getStartDistrib() << "</start>" << std::endl;

        res << "\t\t<actions>" << std::endl;
        for (ag = 0; ag < this->getNumAgents(); ++ag)
            res << "\t\t\t<agent id=\"" << ag << "\">" << this->getActionSpace().getNumElements(ag) << "</agent>" << std::endl;
        res << "\t\t</actions>" << std::endl;

        res << "\t\t<observations>" << std::endl;
        for (ag = 0; ag < this->getNumAgents(); ++ag)
            res << "\t\t\t<agent id=\"" << ag << "\">" << this->getObsSpace().getNumElements(ag) << "</agent>" << std::endl;
        res << "\t\t</observations>" << std::endl;

        res << "\t</preamble>" << std::endl;
        res << "\t<param>" << std::endl;

        number ja;
        res << "\t\t<reward>" << std::endl;
        for (ja = 0; ja < this->getNumJActions(); ++ja)
        {
            std::vector<number> v_ja = this->getActionSpace().getJointElement(ja);
            if (rew_.size() == 1)
            {
                res << "\t\t\t<reward-entry joint-action=\"";
                for (number ag=0; ag < this->getNumAgents(); ++ag)
                {
                    res << this->getActionSpace().getElementName(ag, v_ja[ag]) << " ";
                }
                res << "\" >" << std::endl;
                res << "\t\t\t\t" << this->rew_[0].getReward(ja) << std::endl;
                res << "\t\t\t</reward-entry>" << std::endl;
            }
            else
            {
                for (number ag_id = 0; ag_id < this->getNumAgents(); ++ag_id)
                {
                    res << "\t\t\t<reward-entry joint-action=\"";
                    for (number ag=0; ag < this->getNumAgents(); ++ag)
                    {
                        res << this->getActionSpace().getElementName(ag, v_ja[ag]) << " ";
                    }
                    res << "\" agent=\"" << ag_id << "\">" << std::endl;
                    res << "\t\t\t\t" << this->rew_[ag_id].getReward(ja) << std::endl;
                    res << "\t\t\t</reward-entry>" << std::endl;
                }
            }
        }

        res << "\t\t</reward>" << std::endl;

        observation jz;
        res << "\t\t<dynamics>" << std::endl;
        for (ja = 0; ja < this->getNumJActions(); ++ja)
            for (jz = 0; jz < this->getNumJObservations(); ++jz)
            {
                res << "\t\t\t<dynamics-entry jaction=\"" << ja << "\" jobservation=\"" << jz << "\">" << std::endl;
                res << "\t\t\t\t" << this->getObsDynamics().getDynamics(ja, jz) << std::endl;
                res << "\t\t\t</dynamics-entry>" << std::endl;
            }
        res << "\t\t</dynamics>" << std::endl;

        res << "\t</param>" << std::endl;
        res << "</decpomdp>" << std::endl;

        return res.str();
    }

    std::string POSG::toJSON() const
    {
        std::cout << "toJSON : Not implemented method" << std::endl;
        return "Not implemented method";
    }


    void POSG::generateFile(std::string filename) const
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

} // namespace sdm
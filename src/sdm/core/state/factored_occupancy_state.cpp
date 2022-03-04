
#include <sdm/core/state/factored_occupancy_state.hpp>

namespace sdm
{
    FactoredOccupancyState::FactoredOccupancyState(int memory, const std::shared_ptr<NetworkedDistributedPOMDPInterface> &ndpomdp)
    {
        std::vector<std::shared_ptr<State>> sub_states;

        for (auto i = 0; i < ndpomdp->getNumAgents(); ++i)
        {
            auto belief = ndpomdp->getBeliefState(i);                     // TODO define NetworkedDistributedPOMDPInterface::getBeliefState(int agent)
            auto history = std::make_shared<JointHistoryTree>(1, memory); // TODO to be checked
            auto occupancy = std::make_shared<OccupancyState>(1, 0);
            occupancy->setProbability(history, belief, 1.0);
            occupancy->finalize();
            sub_states.push_back(occupancy);
        }

        this->ndpomdp = ndpomdp;
        this->sub_states = sub_states;
    }

    FactoredOccupancyState::FactoredOccupancyState(const std::vector<std::shared_ptr<State>> &sub_states)
    {
        this->sub_states = sub_states;
    }

    double FactoredOccupancyState::product(const std::shared_ptr<AlphaVector> &alpha)
    {
        auto value = 0.0, group_value;

        std::shared_ptr<Factored_oAlpha> sub_alphas = std::static_pointer_cast<Factored_oAlpha>(alpha); // TODO defined FactoredoAlpha, a tuple of alpha-vectors, one per group of agents in NDPOMDPs

        for (int group = 0; group < this->ndpomdp->getNumGroups(); ++group)
        {
            for (const auto &pair_o_group_other : sub_alphas->getAlphaGroup(group)) // TODO AlphaVector::getSupports(), accessor of the supports in  alpha-vectors.
            {
                double probability = 1.0;
                auto o_group = pair_o_group_other.first;

                for (const auto &pair_x_group_value : pair_o_group_other.second)
                {
                    auto x_group = pair_x_group_value.first;
                    group_value = pair_x_group_value.second;
                    for (int i : this->ndpomdp->getGroup(group))
                    {
                        probability *= this->sub_states[i]->getProbability(o_group[i], x_group[i]); // TODO check the format of support wrt OccupancyState::getProbability( ... )
                    }
                    value += probability * group_value; // TODO check the format of support in AlphaVector::getValueAt( ... )
                }
            }
        }

        return value;
    }

    double FactoredOccupancyState::product(const std::shared_ptr<BetaVector> &beta, const std::shared_ptr<Action> &action)
    {
        auto qvalue = 0.0;
        auto jdecision_rule = action->toDecisionRule();
        std::shared_ptr<FactoredBetaVector> sub_betas = std::static_pointer_cast<FactoredBetaVector>(beta); // TODO defined FactoredBetaVector, a tuple of beta-vectors, one per group of agents in NDPOMDPs

        for (int group = 0; group < this->ndpomdp->getNumGroups(); ++group)
        {
            for (auto support : sub_betas[group]->getSupports())
            { // TODO BetaVector::getSupports(), accessor of the supports in  alpha-vectors.
                // WARNING
                auto &[o_group, x_group, u_group] = support;

                auto probability = 1.0;
                for (int i : this->ndpomdp->getGroup(group))
                {
                    probability *= jdecision_rule->getProbability(o_group[i], u_group[i], i);
                    probability *= this->sub_states[i]->getProbability(x_group[i], o_group[i]); // TODO check the format of support wrt OccupancyState::getProbability( ... )
                }

                qvalue += probability * sub_betas[group]->getValueAt(o_group, x_group, u_group); // TODO check the format of support in AlphaVector::getValueAt( ... )
            }
        }

        return qvalue;
    }

    Pair<std::shared_ptr<State>, double> FactoredOccupancyState::next(const std::shared_ptr<MDPInterface> &mdp, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t)
    {
        auto jdecision_rule = action->toDecisionRule();
        std::vector<std::shared_ptr<State>> sub_states;

        for (auto i = 0; i < this->ndpomdp->getNumAgents(); ++i)
        {
            sub_states.push_back(sub_states[i]->next(mdp, jdecision_rule[i], observation[i], t)); // TODO check the access to the i-th elements for all decision rules, and observations
        }

        return std::make_shared<FactoredOccupancyState>(sub_states);
    }

    double FactoredOccupancyState::getReward(const std::shared_ptr<MDPInterface> &mdp, const std::shared_ptr<Action> &action, number t)
    {
        auto reward = 0.0;
        auto jdecision_rule = action->toDecisionRule();

        for (int group = 0; group < this->ndpomdp->getNumGroups(); ++group)
        {
            for (auto o_group : this->jdecision_rule->getSupport()) // TODO Gets o_group from jdecision_rule
            {
                for (auto x_group : this->ndpomdp->getNumStates(group)) // TODO Gets x_group from ndpomdp.
                {
                    auto probability = 1.0;
                    for (int i : this->ndpomdp->getGroup(group))
                    {
                        // TODO Gets set of agent from a given group
                        probability *= jdecision_rule->getProbability(o_group[i], u_group[i], i);
                        probability *= this->sub_states[i]->getProbability(x_group[i], o_group[i]);
                    }

                    reward += probability * this->ndpomdp->getReward(group, x_group, u_group); // TODO Gets reward from ndpomdp
                }
            }
        }

        return reward;
    }

    size_t FactoredOccupancyState::hash(double precision) const
    {
        size_t seed = 0;
        for (auto i = 0; i < this->ndpomdp->getNumAgents(); ++i)
        {
            seed ^= sub_states[i]->hash(precision) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        }
        return seed;
    }

    bool FactoredOccupancyState::isEqual(const FactoredOccupancyState &other, double precision) const
    {
        for (auto i = 0; i < this->ndpomdp->getNumAgents(); ++i)
        {
            if (!this->sub_states[i]->isEqual(other.sub_states[i], precision))
            {
                return false;
            }
        }
        return true;
    }

    bool FactoredOccupancyState::isEqual(const std::shared_ptr<State> &other, double precision) const
    {
        return this->isEqual(*std::dynamic_pointer_cast<FactoredOccupancyState>(other), precision);
    }

    std::string FactoredOccupancyState::str() const
    {
        std::ostringstream res;
        res << std::setprecision(config::OCCUPANCY_DECIMAL_PRINT) << std::fixed;

        res << "<factored-occupancy-state>\n";
        for (auto i = 0; i < this->ndpomdp->getNumAgents(); ++i)
        {
            res << this->sub_states[i]->str() << "\n";
        }
        res << "</factored-occupancy-state>";
        return res.str();
    }

}
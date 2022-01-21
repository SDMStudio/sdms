#include <math.h>
#include <sdm/utils/value_function/qfunction/pwlc_qvalue_function.hpp>
#include <sdm/core/action/decision_rule.hpp>
#include <sdm/core/state/occupancy_state.hpp>
#include <sdm/core/state/belief_state.hpp>
#include <sdm/world/occupancy_mdp.hpp>

namespace sdm
{
    double PWLCQValueFunction::GRANULARITY = 0.1;
    double PWLCQValueFunction::G = PWLCQValueFunction::GRANULARITY;

    PWLCQValueFunction::PWLCQValueFunction(const std::shared_ptr<SolvableByDP> &world,
                                           const std::shared_ptr<Initializer> &initializer,
                                           const std::shared_ptr<ActionSelectionInterface> &action_selection,
                                           const std::shared_ptr<PWLCQUpdateOperator> &update_operator)
        : ValueFunctionInterface(world, initializer, action_selection),
          QValueFunction(world, initializer, action_selection, update_operator),
          PWLCValueFunctionInterface(world, initializer, action_selection)
    {
        // Create all different structure in order to use the hyperplan q-value function.
        this->representation = std::vector<Container>(this->isInfiniteHorizon() ? 1 : this->getHorizon() + 1, Container());
        this->default_values_per_horizon = std::vector<double>(this->isInfiniteHorizon() ? 1 : world->getHorizon() + 1, 0);
        this->default_hyperplane = std::vector<std::shared_ptr<Hyperplane>>(this->isInfiniteHorizon() ? 1 : world->getHorizon() + 1, nullptr);
        this->oMDP = std::dynamic_pointer_cast<OccupancyMDP>(getWorld());

        for (int t = 0; t < this->oMDP->getHorizon(); t++)
        {
            double granul_t = PWLCQValueFunction::GRANULARITY + sqrt(float(t) / (this->oMDP->getHorizon() - 1)) * (1 - PWLCQValueFunction::GRANULARITY);
            // double granul_t = PWLCQValueFunction::GRANULARITY + t * (1. - PWLCQValueFunction::GRANULARITY) / (this->oMDP->getHorizon() - 1); // LINEAIRE
            std::cout << "t " << t << " - g " << granul_t << std::endl;
            granularity_per_horizon.push_back(granul_t);
        }
        granularity_per_horizon.push_back(1);
    }

    void PWLCQValueFunction::initialize(double value, number t)
    {
        // If there are not element at time t, we have to create the default State
        if (this->representation[t].size() == 0)
        {
            this->default_values_per_horizon[t] = value;
            // Setup default hyperplane
            this->default_hyperplane[t] = std::make_shared<Hyperplane>(value);
        }
    }

    void PWLCQValueFunction::initialize()
    {
        this->initializer_->init(this->getptr());
    }

    double PWLCQValueFunction::getQValueAt(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t)
    {
        // Update the hyperplane of the ball that include the sampled state
        if (auto ostate = sdm::isInstanceOf<OccupancyStateInterface>(state))
            return this->getQValueAt(ostate, action->toDecisionRule(), t);
        else
            throw sdm::exception::TypeError("TypeError: state must derive from OccupancyStateInterface");
    }

    double PWLCQValueFunction::getQValueAt(const std::shared_ptr<OccupancyStateInterface> &occupancy_state, const std::shared_ptr<DecisionRule> &decision_rule, number t)
    {
        double qvalue = 0;

        auto hyperplane = this->getHyperplaneAt(occupancy_state, t);
        if (hyperplane == this->default_hyperplane[t])
        {
            qvalue = this->getDefaultValue(t);
        }
        else
        {
            for (auto history : occupancy_state->getJointHistories())
            {
                auto action = this->oMDP->applyDecisionRule(occupancy_state, history, decision_rule, this->isInfiniteHorizon() ? 0 : t);
                auto dr_input = this->oMDP->getDecisionRuleInput(history, t);
                double proba_a = decision_rule->getProbability(dr_input, action);

                for (auto state : occupancy_state->getBeliefAt(history)->getStates())
                {
                    double beta = this->getBeta(hyperplane, state, history, action, t);
                    qvalue += occupancy_state->getProbability(history, state) * proba_a * beta;
                }
            }
        }
        return qvalue;
    }

    void PWLCQValueFunction::addHyperplaneAt(const std::shared_ptr<State> &state, const std::shared_ptr<State> &new_hyperplane, number t)
    {
        PWLCQValueFunction::G = this->granularity_per_horizon[t];
        this->representation[t][state] = std::dynamic_pointer_cast<Hyperplane>(new_hyperplane);
    }

    std::shared_ptr<State> PWLCQValueFunction::getHyperplaneAt(const std::shared_ptr<State> &state, number t)
    {
        PWLCQValueFunction::G = this->granularity_per_horizon[t];
        auto tmp_it = this->representation[t].find(state);
        if (tmp_it == this->representation[t].end())
        {
            auto new_hyperplane = std::make_shared<Hyperplane>(this->default_values_per_horizon[t]);
            this->addHyperplaneAt(state, new_hyperplane, t);
            return new_hyperplane;
        }
        else
        {
            return tmp_it->second;
        }
    }

    std::vector<std::shared_ptr<State>> PWLCQValueFunction::getHyperplanesAt(const std::shared_ptr<State> &state, number t)
    {
        return {getHyperplaneAt(state, t - 1)};
    }

    double PWLCQValueFunction::getBeta(const std::shared_ptr<State> &hyperplane, const std::shared_ptr<State> &x, const std::shared_ptr<HistoryInterface> &o, const std::shared_ptr<Action> &u, number t)
    {
        return std::static_pointer_cast<Hyperplane>(hyperplane)->getValueAt(o, x, u);
    }

    double PWLCQValueFunction::getDefaultValue(number t)
    {
        return this->default_values_per_horizon[t];
    }

    void PWLCQValueFunction::prune(number t) {}

    PWLCQValueFunction::Container PWLCQValueFunction::getRepresentation(number t)
    {
        return this->representation[this->isInfiniteHorizon() ? 0 : t];
    }

    std::string PWLCQValueFunction::str() const
    {
        std::ostringstream res;
        res << "<pwlc_qvalue_function horizon=\"" << ((this->isInfiniteHorizon()) ? "inf" : std::to_string(this->getHorizon())) << "\">" << std::endl;
        for (sdm::size_t i = 0; i < this->representation.size(); i++)
        {
            res << "\t<value timestep=\"" << ((this->isInfiniteHorizon()) ? "all" : std::to_string(i)) << ">" << std::endl;
            for (const auto &pair_state_hyperplane : this->representation[i])
            {

                res << "\t\t<state>" << std::endl;
                tools::indentedOutput(res, pair_state_hyperplane.first->str().c_str(), 3);
                res << "\t\t</state>" << std::endl;
                res << "\t\t<plan>" << std::endl;

                std::ostringstream hyperplan_str;
                hyperplan_str << pair_state_hyperplane.second->getState();
                tools::indentedOutput(res, hyperplan_str.str().c_str(), 3);

                res << "\n\t\t</plan>" << std::endl;
            }
            res << "\t</value>" << std::endl;
        }
        res << "</pwlc_qvalue_function>" << std::endl;
        return res.str();
    }
} // namespace sdm
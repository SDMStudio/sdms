#include <sdm/utils/value_function/qfunction/parametric_qvalue_function.hpp>
#include <sdm/utils/value_function/qfunction/tabular_qvalue_function.hpp>

namespace sdm
{

    ParametricQValueFunction::ParametricQValueFunction(const std::shared_ptr<SolvableByDP> &world,
                                                       const std::shared_ptr<Initializer> &initializer,
                                                       const std::shared_ptr<ActionSelectionInterface> &action,
                                                       const std::shared_ptr<PWLCQUpdateOperator> &update_operator)
        : ValueFunctionInterface(world, initializer, action),
          QValueFunction(world, initializer, action, update_operator),
          PWLCValueFunctionInterface(world, initializer, action)
    {
        // Create all different structure in order to use the hyperplan q-value function.
        this->representation = std::vector<Container>(this->isInfiniteHorizon() ? 1 : this->getHorizon() + 1, Container());
        this->default_values_per_horizon = std::vector<double>(this->isInfiniteHorizon() ? 1 : world->getHorizon() + 1, 0);
        this->oMDP = std::dynamic_pointer_cast<OccupancyMDP>(getWorld());
    }

    void ParametricQValueFunction::initialize()
    {
        this->initializer_->init(this->getptr());
    }

    void ParametricQValueFunction::initialize(double value, number t)
    {
        // If there are not element at time t, we have to create the default State
        this->default_values_per_horizon[t] = value;
        this->representation[t] = std::make_shared<Hyperplane>(value);
    }

    double ParametricQValueFunction::getQValueAt(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t)
    {
        double qvalue = 0.0;

        auto occupancy_state = state->toOccupancyState();
        auto decision_rule = action->toDecisionRule();

        for (auto history : occupancy_state->getJointHistories())
        {
            auto action = this->oMDP->applyDecisionRule(occupancy_state, history, decision_rule, this->isInfiniteHorizon() ? 0 : t);
            auto dr_input = this->oMDP->getDecisionRuleInput(history, t);
            double proba_a = decision_rule->getProbability(dr_input, action);

            for (auto state : occupancy_state->getBeliefAt(history)->getStates())
            {
                double beta = this->getQValueAt(state, history, action, t);
                qvalue += occupancy_state->getProbability(history, state) * proba_a * beta;
            }
        }
        return qvalue;
    }

    double ParametricQValueFunction::getQValueAt(const std::shared_ptr<State> &x, const std::shared_ptr<HistoryInterface> &o, const std::shared_ptr<Action> &u, number t)
    {
        return this->representation[t]->getValueAt(o, x, u);
    }

    void ParametricQValueFunction::setQValueAt(const std::shared_ptr<State> &x, const std::shared_ptr<HistoryInterface> &o, const std::shared_ptr<Action> &u, double value, number t)
    {
        this->representation[t]->setValueAt(o, x, u, value);
    }

    void ParametricQValueFunction::addHyperplaneAt(const std::shared_ptr<State> &, const std::shared_ptr<State> &new_hyperplane, number t)
    {
        this->representation[t] = std::static_pointer_cast<Hyperplane>(new_hyperplane);
    }

    std::shared_ptr<State> ParametricQValueFunction::getHyperplaneAt(std::shared_ptr<State>, number t)
    {
        return this->representation[t];
    }

    ParametricQValueFunction::Container ParametricQValueFunction::getRepresentation(number t)
    {
        return this->representation[t];
    }

    std::vector<std::shared_ptr<State>> ParametricQValueFunction::getHyperplanesAt(std::shared_ptr<State>, number t)
    {
        return {getHyperplaneAt(nullptr, t - 1)};
    }

    double ParametricQValueFunction::getBeta(const std::shared_ptr<State> &, const std::shared_ptr<State> &state, const std::shared_ptr<HistoryInterface> &history, const std::shared_ptr<Action> &action, number t)
    {
        return this->getQValueAt(state, history, action, t);
    }

    double ParametricQValueFunction::getDefaultValue(number t)
    {
        return this->default_values_per_horizon[t];
    }

    void ParametricQValueFunction::prune(number t) {}

    std::string ParametricQValueFunction::str() const
    {
        std::ostringstream res;
        res << "<param_qvalue_function horizon=\"" << ((this->isInfiniteHorizon()) ? "inf" : std::to_string(this->getHorizon())) << "\">" << std::endl;
        for (sdm::size_t i = 0; i < this->representation.size(); i++)
        {
            res << "\t\t<plan timestep=\"" << ((this->isInfiniteHorizon()) ? "all" : std::to_string(i)) << ">" << std::endl;

            std::ostringstream hyperplan_str;
            hyperplan_str << this->representation[i]->getState();
            tools::indentedOutput(res, hyperplan_str.str().c_str(), 3);

            res << "\n\t\t</plan>" << std::endl;
        }
        res << "</pwlc_qvalue_function>" << std::endl;
        return res.str();
    }

}
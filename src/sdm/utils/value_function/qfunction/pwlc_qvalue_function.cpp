#include <sdm/utils/value_function/qfunction/pwlc_qvalue_function.hpp>
#include <sdm/core/action/decision_rule.hpp>
#include <sdm/core/state/occupancy_state.hpp>
#include <sdm/core/state/belief_state.hpp>
#include <sdm/world/occupancy_mdp.hpp>

namespace sdm
{
    double PWLCQValueFunction::GRANULARITY = 0.1;

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
    }

    void PWLCQValueFunction::initialize(double value, number t)
    {

        // If there are not element at time t, we have to create the default State
        if (this->representation[t].size() == 0)
        {
            this->default_values_per_horizon[t] = value;
            // Setup default hyperplane
            this->default_hyperplane[t] = std::make_shared<Hyperplane>(MappedVector<Tuple<std::shared_ptr<State>, std::shared_ptr<HistoryInterface>, std::shared_ptr<Action>>>(value));
        }
    }

    void PWLCQValueFunction::initialize()
    {
        this->initializer_->init(this->getptr());
    }

    double PWLCQValueFunction::getQValueAt(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t)
    {
        // Update the hyperplane of the ball that include the sampled state
        if (sdm::isInstanceOf<OccupancyStateInterface>(state))
        {
            return this->getQValueAt(state->toOccupancyState(), action->toDecisionRule(), t);
        }
        else
        {
            throw sdm::exception::TypeError("TypeError: state must derive from OccupancyStateInterface");
        }
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
                auto action = std::dynamic_pointer_cast<OccupancyMDP>(getWorld())->applyDecisionRule(occupancy_state, history, decision_rule, this->isInfiniteHorizon() ? 0 : t);

                for (auto state : occupancy_state->getBeliefAt(history)->getStates())
                {
                    qvalue += occupancy_state->getProbability(history, state) /* * decision_rule->getProbability(history, action) */ * this->getBeta(hyperplane, state, history, action, t);
                    // hyperplane->getValue(state, history, action);
                }
            }
        }
        return qvalue;
    }

    void PWLCQValueFunction::addHyperplaneAt(const std::shared_ptr<State> &state, const std::shared_ptr<State> &new_hyperplane, number t)
    {
        this->representation[t][state] = std::dynamic_pointer_cast<Hyperplane>(new_hyperplane);
    }

    std::shared_ptr<State> PWLCQValueFunction::getHyperplaneAt(const std::shared_ptr<State> &state, number t)
    {
        auto tmp_it = this->representation[t].find(state);
        return (tmp_it != this->representation[t].end()) ? tmp_it->second : this->default_hyperplane[t];
    }

    std::vector<std::shared_ptr<State>> PWLCQValueFunction::getHyperplanesAt(number t)
    {
        std::vector<std::shared_ptr<State>> list_hyperplanes;
        for (const auto &pair_state_hyperplane : this->getRepresentation(t))
        {
            list_hyperplanes.push_back(pair_state_hyperplane.second);
        }
        return list_hyperplanes;
    }

    double PWLCQValueFunction::getBeta(const std::shared_ptr<State> &hyperplane, const std::shared_ptr<State> &state, const std::shared_ptr<HistoryInterface> &history, const std::shared_ptr<Action> &action, number t)
    {
        return std::dynamic_pointer_cast<Hyperplane>(hyperplane)->getState().getValueAt(std::tuple(state, history, action));
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
        // res << "<hierarchical_qvalue_function_v2 horizon=\"" << ((this->isInfiniteHorizon()) ? "inf" : std::to_string(this->getHorizon())) << "\">" << std::endl;
        // for (sdm::size_t i = 0; i < this->representation.size(); i++)
        // {
        //     res << "\t<timestep=\"" << ((this->isInfiniteHorizon()) ? "all" : std::to_string(i)) << "\" default=\"" << 0 << "\">" << std::endl;
        //     for (auto const& [s, q] : this->representation[i])
        //     {
        //         auto jh = *s->getJointHistories().begin();
        //         if (jh->getHorizon() < this->horizon_)
        //         {
        //             res << "\t\t<S-Q>" << std::endl;
        //             tools::indentedOutput(res, s->str().c_str(), 3);
        //             res << std::endl;
        //             tools::indentedOutput(res, q.str().c_str(), 3);
        //             res << "\t\t</S-Q>" << std::endl;
        //         }
        //     }
        //     res << "\t</timestep>" << std::endl;
        // }
        // res << "</hierarchical_qvalue_function_v2>" << std::endl;
        return res.str();
    }
} // namespace sdm
#include <math.h>
#include <sdm/utils/value_function/qfunction/pwlc_qvalue_function.hpp>
#include <sdm/core/action/decision_rule.hpp>
#include <sdm/core/state/occupancy_state.hpp>
#include <sdm/core/state/belief_state.hpp>
#include <sdm/world/occupancy_mdp.hpp>
#include <sdm/utils/linear_algebra/hyperplane/obeta.hpp>

namespace sdm
{
    template <typename TBetaVector>
    double PWLCQValueFunction<TBetaVector>::GRANULARITY_START = 0.1;

    template <typename TBetaVector>
    double PWLCQValueFunction<TBetaVector>::GRANULARITY_END = 1.0;

    template <typename TBetaVector>
    double PWLCQValueFunction<TBetaVector>::GRANULARITY = PWLCQValueFunction<TBetaVector>::GRANULARITY_START;

    template <typename TBetaVector>
    PWLCQValueFunction<TBetaVector>::PWLCQValueFunction(const std::shared_ptr<SolvableByDP> &world,
                                           const std::shared_ptr<Initializer> &initializer,
                                           const std::shared_ptr<ActionSelectionInterface> &action_selection,
                                           const std::shared_ptr<PWLCQUpdateOperator> &update_operator)
        : ValueFunctionInterface(world, initializer, action_selection),
          QValueFunction(world, initializer, action_selection, update_operator),
          PWLCValueFunctionInterface(world, initializer, action_selection)
    {
        // Create all different structure in order to use the hyperplan q-value function.
        this->representation = std::vector<Container>(this->isInfiniteHorizon() ? 1 : world->getHorizon() + 1, Container());
        this->default_values_per_horizon = std::vector<double>(this->isInfiniteHorizon() ? 1 : world->getHorizon() + 1, 0);
        this->default_hyperplane = std::vector<std::shared_ptr<BetaVector>>(this->isInfiniteHorizon() ? 1 : world->getHorizon() + 1, nullptr);

        number num_serial_agents = (sdm::isInstanceOf<SerialProblemInterface>(getWorld()->getUnderlyingProblem())) ? getWorld()->getUnderlyingProblem()->getNumAgents() : 1;
        number real_horizon = getWorld()->getHorizon() / num_serial_agents;
        for (int t = 0; t < (this->isInfiniteHorizon() ? 1 : getWorld()->getHorizon()); t++)
        {
            number real_current_horizon = t / num_serial_agents;

            double granul_t = PWLCQValueFunction::GRANULARITY_START + float(real_current_horizon) / (real_horizon - 1) * (PWLCQValueFunction::GRANULARITY_END - PWLCQValueFunction<TBetaVector>::GRANULARITY_START);
            std::cout << "t " << t << " - g " << granul_t << std::endl;
            granularity_per_horizon.push_back(granul_t);
        }
        granularity_per_horizon.push_back(1);
    }

    template <typename TBetaVector>
    void PWLCQValueFunction<TBetaVector>::initialize(double value, number t)
    {
        if (this->representation[this->isInfiniteHorizon() ? 0 : t].size() == 0)
        {
            this->default_values_per_horizon[this->isInfiniteHorizon() ? 0 : t] = value;
            // Setup default hyperplane
            this->default_hyperplane[this->isInfiniteHorizon() ? 0 : t] = std::make_shared<TBetaVector>(value);
        }
    }

    template <typename TBetaVector>
    void PWLCQValueFunction<TBetaVector>::initialize()
    {
        this->initializer_->init(this->getptr());
    }

    template <typename TBetaVector>
    double PWLCQValueFunction<TBetaVector>::getQValueAt(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t)
    {
        double qvalue = 0;

        PWLCQValueFunction::GRANULARITY = this->granularity_per_horizon[this->isInfiniteHorizon() ? 0 : t];
        auto hyperplane_iter = this->representation[this->isInfiniteHorizon() ? 0 : t].find(state);
        if (hyperplane_iter == this->representation[this->isInfiniteHorizon() ? 0 : t].end())
        {
            auto new_hyperplane = std::make_shared<TBetaVector>(this->default_values_per_horizon[this->isInfiniteHorizon() ? 0 : t]);
            this->addHyperplaneAt(state, new_hyperplane, t);
            qvalue = this->getDefaultValue(t);
        }
        else
        {
            qvalue = state->product(hyperplane_iter->second, action);
        }
        return qvalue;
    }

    template <typename TBetaVector>
    void PWLCQValueFunction<TBetaVector>::addHyperplaneAt(const std::shared_ptr<State> &state, const std::shared_ptr<Hyperplane> &new_hyperplane, number t)
    {
        PWLCQValueFunction::GRANULARITY = this->granularity_per_horizon[this->isInfiniteHorizon() ? 0 : t];
        this->representation[this->isInfiniteHorizon() ? 0 : t][state] = std::static_pointer_cast<BetaVector>(new_hyperplane);
    }

    template <typename TBetaVector>
    std::shared_ptr<Hyperplane> PWLCQValueFunction<TBetaVector>::getHyperplaneAt(std::shared_ptr<State> state, number t)
    {
        PWLCQValueFunction::GRANULARITY = this->granularity_per_horizon[this->isInfiniteHorizon() ? 0 : t];
        auto tmp_it = this->representation[this->isInfiniteHorizon() ? 0 : t].find(state);
        if (tmp_it == this->representation[this->isInfiniteHorizon() ? 0 : t].end())
        {
            auto new_hyperplane = std::make_shared<TBetaVector>(this->default_values_per_horizon[this->isInfiniteHorizon() ? 0 : t]);
            this->addHyperplaneAt(state, new_hyperplane, t);
            return new_hyperplane;
        }
        else
        {
            state = tmp_it->first;
            return tmp_it->second;
        }
    }

    template <typename TBetaVector>
    std::vector<std::shared_ptr<Hyperplane>> PWLCQValueFunction<TBetaVector>::getHyperplanesAt(std::shared_ptr<State> state, number t)
    {
        return {getHyperplaneAt(state, t - 1)};
    }

    template <typename TBetaVector>
    double PWLCQValueFunction<TBetaVector>::getBeta(const std::shared_ptr<Hyperplane> &hyperplane, const std::shared_ptr<State> &x, const std::shared_ptr<HistoryInterface> &o, const std::shared_ptr<Action> &u, number t)
    {
        return hyperplane->getValueAt(x, o, u);
    }

    template <typename TBetaVector>
    double PWLCQValueFunction<TBetaVector>::getDefaultValue(number t)
    {
        return this->default_values_per_horizon[this->isInfiniteHorizon() ? 0 : t];
    }

    template <typename TBetaVector>
    void PWLCQValueFunction<TBetaVector>::prune(number t) {}

    template <typename TBetaVector>
    typename PWLCQValueFunction<TBetaVector>::Container PWLCQValueFunction<TBetaVector>::getRepresentation(number t)
    {
        return this->representation[this->isInfiniteHorizon() ? 0 : t];
    }

    template <typename TBetaVector>
    std::string PWLCQValueFunction<TBetaVector>::str() const
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
                res << "\n\t\t</state>" << std::endl;
                std::ostringstream hyperplan_str;
                hyperplan_str << pair_state_hyperplane.second->str();
                tools::indentedOutput(res, hyperplan_str.str().c_str(), 2);
            }
            res << "\t</value>" << std::endl;
        }
        res << "</pwlc_qvalue_function>";
        return res.str();
    }
} // namespace sdm
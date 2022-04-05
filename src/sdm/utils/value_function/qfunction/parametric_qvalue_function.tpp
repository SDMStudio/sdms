#include <sdm/utils/value_function/qfunction/parametric_qvalue_function.hpp>
#include <sdm/utils/value_function/qfunction/tabular_qvalue_function.hpp>
#include <sdm/utils/linear_algebra/hyperplane/obeta.hpp>
#include <sdm/utils/linear_algebra/hyperplane/bbeta.hpp>

namespace sdm
{

    template <typename TBetaVector>
    ParametricQValueFunction<TBetaVector>::ParametricQValueFunction(const std::shared_ptr<SolvableByDP> &world,
                                                       const std::shared_ptr<Initializer> &initializer,
                                                       const std::shared_ptr<ActionSelectionInterface> &action,
                                                       const std::shared_ptr<PWLCQUpdateRule> &update_rule)
        : ValueFunctionInterface(world, initializer, action),
          QValueFunction(world, initializer, action, update_rule),
          PWLCValueFunctionInterface(world, initializer, action)
    {
        // Create all different structure in order to use the hyperplan q-value function.
        this->representation = std::vector<Container>(this->isInfiniteHorizon() ? 1 : this->getHorizon() + 1, Container());
        this->default_values_per_horizon = std::vector<double>(this->isInfiniteHorizon() ? 1 : world->getHorizon() + 1, 0);
    }

    template <typename TBetaVector>
    void ParametricQValueFunction<TBetaVector>::initialize()
    {
        this->initializer_->init(this->getptr());
    }

    template <typename TBetaVector>
    void ParametricQValueFunction<TBetaVector>::initialize(double value, number t)
    {
        // If there are not element at time t, we have to create the default State
        this->default_values_per_horizon[this->isInfiniteHorizon() ? 0 : t] = value;
        this->representation[this->isInfiniteHorizon() ? 0 : t] = std::make_shared<TBetaVector>(value);
    }

    template <typename TBetaVector>
    double ParametricQValueFunction<TBetaVector>::getQValueAt(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t)
    {
        return state->product(this->representation[this->isInfiniteHorizon() ? 0 : t], action);
    }

    template <typename TBetaVector>
    double ParametricQValueFunction<TBetaVector>::getQValueAt(const std::shared_ptr<State> &x, const std::shared_ptr<HistoryInterface> &o, const std::shared_ptr<Action> &u, number t)
    {
        return this->representation[this->isInfiniteHorizon() ? 0 : t]->getValueAt(x, o, u);
    }

    template <typename TBetaVector>
    void ParametricQValueFunction<TBetaVector>::setQValueAt(const std::shared_ptr<State> &x, const std::shared_ptr<HistoryInterface> &o, const std::shared_ptr<Action> &u, double value, number t)
    {
        this->representation[this->isInfiniteHorizon() ? 0 : t]->setValueAt(x, o, u, value);
    }

    template <typename TBetaVector>
    void ParametricQValueFunction<TBetaVector>::addHyperplaneAt(const std::shared_ptr<State> &, const std::shared_ptr<Hyperplane> &new_hyperplane, number t)
    {
        this->representation[this->isInfiniteHorizon() ? 0 : t] = std::static_pointer_cast<BetaVector>(new_hyperplane);
    }

    template <typename TBetaVector>
    std::shared_ptr<Hyperplane> ParametricQValueFunction<TBetaVector>::getHyperplaneAt(std::shared_ptr<State>, number t)
    {
        return this->representation[this->isInfiniteHorizon() ? 0 : t];
    }

    template <typename TBetaVector>
    typename ParametricQValueFunction<TBetaVector>::Container ParametricQValueFunction<TBetaVector>::getRepresentation(number t)
    {
        return this->representation[this->isInfiniteHorizon() ? 0 : t];
    }

    template <typename TBetaVector>
    std::vector<std::shared_ptr<Hyperplane>> ParametricQValueFunction<TBetaVector>::getHyperplanesAt(std::shared_ptr<State>, number t)
    {
        return {getHyperplaneAt(nullptr, t - 1)};
    }

    template <typename TBetaVector>
    double ParametricQValueFunction<TBetaVector>::getBeta(const std::shared_ptr<Hyperplane> &, const std::shared_ptr<State> &state, const std::shared_ptr<HistoryInterface> &history, const std::shared_ptr<Action> &action, number t)
    {
        return this->getQValueAt(state, history, action, t);
    }

    template <typename TBetaVector>
    double ParametricQValueFunction<TBetaVector>::getDefaultValue(number t)
    {
        return this->default_values_per_horizon[this->isInfiniteHorizon() ? 0 : t];
    }

    template <typename TBetaVector>
    void ParametricQValueFunction<TBetaVector>::prune(number t) {}

    template <typename TBetaVector>
    std::string ParametricQValueFunction<TBetaVector>::str() const
    {
        std::ostringstream res;
        res << "<param_qvalue_function horizon=\"" << ((this->isInfiniteHorizon()) ? "inf" : std::to_string(this->getHorizon())) << "\">" << std::endl;
        for (sdm::size_t i = 0; i < this->representation.size(); i++)
        {
            res << "\t\t<plan timestep=\"" << ((this->isInfiniteHorizon()) ? "all" : std::to_string(i)) << ">" << std::endl;

            std::ostringstream hyperplan_str;
            hyperplan_str << this->representation[i]->str();
            tools::indentedOutput(res, hyperplan_str.str().c_str(), 3);

            res << "\n\t\t</plan>" << std::endl;
        }
        res << "</pwlc_qvalue_function>" << std::endl;
        return res.str();
    }

}
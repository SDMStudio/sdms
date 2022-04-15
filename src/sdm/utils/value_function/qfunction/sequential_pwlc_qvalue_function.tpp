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
    double SequentialPWLCQValueFunction<TBetaVector>::GRANULARITY_START = 0.1;

    template <typename TBetaVector>
    double SequentialPWLCQValueFunction<TBetaVector>::GRANULARITY_END = 1.0;

    template <typename TBetaVector>
    double SequentialPWLCQValueFunction<TBetaVector>::GRANULARITY = SequentialPWLCQValueFunction<TBetaVector>::GRANULARITY_START;

    template <typename TBetaVector>
    SequentialPWLCQValueFunction<TBetaVector>::SequentialPWLCQValueFunction(const std::shared_ptr<SolvableByDP> &world,
                                                                            const std::shared_ptr<Initializer> &initializer,
                                                                            const std::shared_ptr<ActionSelectionInterface> &action_selection,
                                                                            const std::shared_ptr<PWLCQUpdateRule> &update_rule)
        : ValueFunctionInterface(world, initializer, action_selection),
          QValueFunction(world, initializer, action_selection, update_rule),
          PWLCValueFunctionInterface(world, initializer, action_selection)
    {
        // Create all different structure in order to use the hyperplan q-value function.
        number num_serial_agents = (sdm::isInstanceOf<SerialProblemInterface>(getWorld()->getUnderlyingProblem())) ? getWorld()->getUnderlyingProblem()->getNumAgents() : 1;
        number real_horizon = getWorld()->getHorizon() / num_serial_agents;
        this->simultaneous_representation = std::vector<SimultaneousContainer>(this->isInfiniteHorizon() ? 1 : real_horizon + 1, Container());
        this->sequential_representation = std::vector<SequentialContainer>(this->isInfiniteHorizon() ? 1 : real_horizon * (world->getNumAgents() - 1), Container());
        this->default_values_per_horizon = std::vector<double>(this->isInfiniteHorizon() ? 1 : world->getHorizon() + 1, 0);
        this->default_hyperplane = std::vector<std::shared_ptr<BetaVector>>(this->isInfiniteHorizon() ? 1 : world->getHorizon() + 1, nullptr);

        for (int t = 0; t < (this->isInfiniteHorizon() ? 1 : getWorld()->getHorizon()); t++)
        {
            number real_current_horizon = t / num_serial_agents;

            double granul_t = SequentialPWLCQValueFunction::GRANULARITY_START + float(real_current_horizon) / (real_horizon - 1) * (SequentialPWLCQValueFunction::GRANULARITY_END - SequentialPWLCQValueFunction<TBetaVector>::GRANULARITY_START);
            std::cout << "t " << t << " - g " << granul_t << std::endl;
            granularity_per_horizon.push_back(granul_t);
        }
        granularity_per_horizon.push_back(1);
    }

    template <typename TBetaVector>
    void SequentialPWLCQValueFunction<TBetaVector>::initialize(double value, number t)
    {
        bool is_initialized = (this->isLastAgent(t)) ? (this->simultaneous_representation[getSimIndex(t)].size() == 0) : (this->sequential_representation[getSerialIndex(t)].size() == 0);
        if (is_initialized)
        {
            this->default_values_per_horizon[this->isInfiniteHorizon() ? 0 : t] = value;
            // Setup default hyperplane
            this->default_hyperplane[this->isInfiniteHorizon() ? 0 : t] = std::make_shared<TBetaVector>(value);
        }
    }
}

template <typename TBetaVector>
void SequentialPWLCQValueFunction<TBetaVector>::initialize()
{
    this->initializer_->init(this->getptr());
}

template <typename TBetaVector>
double SequentialPWLCQValueFunction<TBetaVector>::getQValueAt(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t)
{
    double qvalue = 0;
    bool already_exist = false;

    SequentialPWLCQValueFunction::GRANULARITY = this->granularity_per_horizon[this->isInfiniteHorizon() ? 0 : t];

    if (this->isLastAgent(t))
    {
        auto hyperplane_iter = this->simultaneous_representation[this->getSimIndex(t)].find(state);
        auto end_iter = this->simultaneous_representation[this->getSimIndex(t)].end();
        if (already_exist)
        {
            auto new_hyperplane = std::make_shared<TBetaVector>(this->default_values_per_horizon[this->isInfiniteHorizon() ? 0 : t]);
            this->addHyperplaneAt(state, new_hyperplane, t);
            qvalue = this->getDefaultValue(t);
        }
        else
        {
            qvalue = state->product(hyperplane_iter->second, action);
        }
    }
    else
    {
        auto hyperplane_iter = this->sequential_representation[this->getSerialIndex(t)].find(state);
        auto end_iter = this->sequential_representation[this->getSerialIndex(t)].end();
        if (already_exist)
        {
            auto new_hyperplane = std::make_shared<TBetaVector>(this->default_values_per_horizon[this->isInfiniteHorizon() ? 0 : t]);
            this->addHyperplaneAt(state, new_hyperplane, t);
            qvalue = this->getDefaultValue(t);
        }
        else
        {
            qvalue = state->product(hyperplane_iter->second, action);
        }
    }
    return qvalue;
}

template <typename TBetaVector>
void SequentialPWLCQValueFunction<TBetaVector>::addHyperplaneAt(const std::shared_ptr<State> &state, const std::shared_ptr<Hyperplane> &new_hyperplane, number t)
{
    SequentialPWLCQValueFunction::GRANULARITY = this->granularity_per_horizon[this->isInfiniteHorizon() ? 0 : t];
    if (this->isLastAgent(t))
        this->simultaneous_representation[this->getSimIndex(t)][state] = std::static_pointer_cast<BetaVector>(new_hyperplane);
    else
        this->sequential_representation[this->getSerialIndex(t)][state] = std::static_pointer_cast<BetaVector>(new_hyperplane);
}

template <typename TBetaVector>
std::shared_ptr<Hyperplane> SequentialPWLCQValueFunction<TBetaVector>::getHyperplaneAt(std::shared_ptr<State> state, number t)
{
    SequentialPWLCQValueFunction::GRANULARITY = this->granularity_per_horizon[this->isInfiniteHorizon() ? 0 : t];
    if (isLastAgent(t))
    {
        auto tmp_it = this->simultaneous_representation[this->getSimIndex(t)].find(state);
        if (tmp_it == this->simultaneous_representation[this->getSimIndex(t)].end())
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
    else
    {
        auto tmp_it = this->sequential_representation[this->getSerialIndex(t)].find(state);
        if (tmp_it == this->sequential_representation[this->getSerialIndex(t)].end())
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
}

template <typename TBetaVector>
std::vector<std::shared_ptr<Hyperplane>> SequentialPWLCQValueFunction<TBetaVector>::getHyperplanesAt(std::shared_ptr<State> state, number t)
{
    return {getHyperplaneAt(state, t - 1)};
}

template <typename TBetaVector>
double SequentialPWLCQValueFunction<TBetaVector>::getBeta(const std::shared_ptr<Hyperplane> &hyperplane, const std::shared_ptr<State> &x, const std::shared_ptr<HistoryInterface> &o, const std::shared_ptr<Action> &u, number t)
{
    return hyperplane->getValueAt(x, o, u);
}

template <typename TBetaVector>
double SequentialPWLCQValueFunction<TBetaVector>::getDefaultValue(number t)
{
    return this->default_values_per_horizon[this->isInfiniteHorizon() ? 0 : t];
}

template <typename TBetaVector>
bool SequentialPWLCQValueFunction<TBetaVector>::isLastAgent(number t)
{
    return ((t % this->getWorld()->getNumAgents()) == (this->getWorld()->getNumAgents() - 1));
}

template <typename TBetaVector>
number SequentialPWLCQValueFunction<TBetaVector>::getSimIndex(number t)
{
    return (t / this->getWorld()->getNumAgents());
}

template <typename TBetaVector>
number SequentialPWLCQValueFunction<TBetaVector>::getSerialIndex(number t)
{
    return (t - this->getSimIndex(t));
}


template <typename TBetaVector>
void SequentialPWLCQValueFunction<TBetaVector>::prune(number t) {}

template <typename TBetaVector>
typename SequentialPWLCQValueFunction<TBetaVector>::SimultaneousContainer SequentialPWLCQValueFunction<TBetaVector>::getRepresentation(number t)
{
    return this->simultaneous_representation[this->getSimIndex(t)];
}

template <typename TBetaVector>
std::string SequentialPWLCQValueFunction<TBetaVector>::str() const
{
    std::ostringstream res;
    res << "<pwlc_qvalue_function horizon=\"" << ((this->isInfiniteHorizon()) ? "inf" : std::to_string(this->getHorizon())) << "\">" << std::endl;
    for (sdm::size_t i = 0; i < this->representation.size(); i++)
    {
        res << "\t<value timestep=\"" << ((this->isInfiniteHorizon()) ? "all" : std::to_string(i)) << ">" << std::endl;
        for (const auto &pair_state_hyperplane : this->simultaneous_representation[i])
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
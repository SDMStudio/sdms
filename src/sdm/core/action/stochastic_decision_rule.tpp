#include <sdm/core/action/stochastic_decision_rule.hpp>

namespace sdm
{
    template <typename TState, typename TAction>
    StochasticDecisionRule<TState, TAction>::StochasticDecisionRule() {}

    template <typename TState, typename TAction>
    TAction StochasticDecisionRule<TState, TAction>::act(const TState &) const
    {
        // return this->at(s);
        throw sdm::exception::NotImplementedException();
    }

    template <typename TState, typename TAction>
    TAction StochasticDecisionRule<TState, TAction>::operator()(const TState &s)
    {
        // return this->at(s);
        throw sdm::exception::NotImplementedException();
    }

    template <typename TState, typename TAction>
    RecursiveMap<TAction, double> StochasticDecisionRule<TState, TAction>::getProbabilities(const TState &state)
    {
        return (*this)[state];
    }

    template <typename TState, typename TAction>
    double StochasticDecisionRule<TState, TAction>::getProbability(const TState &state, const TAction &action)
    {
        return (*this)[state][action];
    }

    template <typename TState, typename TAction>
    void StochasticDecisionRule<TState, TAction>::setProbability(const TState &state, const TAction &action, double proba)
    {
        (*this)[state][action] = proba;
    }

} // namespace sdm

namespace std
{
    template <typename S, typename A>
    struct hash<sdm::StochasticDecisionRule<S, A>>
    {
        typedef sdm::StochasticDecisionRule<S, A> argument_type;
        typedef std::size_t result_type;
        inline result_type operator()(const argument_type &in) const
        {
            size_t seed = 0;
            for (auto &input : in)
            {
                sdm::hash_combine(seed, input.first);
                sdm::hash_combine(seed, input.second);
            }
            return seed;
        }
    };
}
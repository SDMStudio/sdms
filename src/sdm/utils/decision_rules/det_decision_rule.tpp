#include <sdm/utils/decision_rules/det_decision_rule.hpp>

namespace sdm
{
    template <typename TState, typename TAction>
    DeterministicDecisionRule<TState, TAction>::DeterministicDecisionRule() {}

    template <typename TState, typename TAction>
    DeterministicDecisionRule<TState, TAction>::DeterministicDecisionRule(std::vector<TState> acc_states, std::vector<TAction> n_actions)
    {
        assert(acc_states.size() == n_actions.size());
        for (std::size_t i = 0; i < acc_states.size(); i++)
        {
            (*this)[acc_states[i]] = n_actions[i];
        }
    }

    template <typename TState, typename TAction>
    TAction DeterministicDecisionRule<TState, TAction>::act(const TState &s) const
    {
        return this->at(s);
    }

    template <typename TState, typename TAction>
    TAction DeterministicDecisionRule<TState, TAction>::operator()(const TState &s)
    {
        return this->at(s);
    }
    
} // namespace sdm

namespace std
{
    template <typename S, typename A>
    struct hash<sdm::DeterministicDecisionRule<S, A>>
    {
        typedef sdm::DeterministicDecisionRule<S, A> argument_type;
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
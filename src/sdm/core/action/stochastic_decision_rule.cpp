#include <sdm/core/action/stochastic_decision_rule.hpp>
#include <sdm/exception.hpp>
#include <sdm/tools.hpp>


namespace sdm
{
    StochasticDecisionRule::StochasticDecisionRule() {}

    std::shared_ptr<Action> StochasticDecisionRule::act(const std::shared_ptr<State> &) const
    {
        // return this->at(s);
        throw sdm::exception::NotImplementedException("NotImplementedException raised in StochasticDecisionRule::act");
    }

    // template <typename std::shared_ptr<State>, typename std::shared_ptr<Action>>
    // std::shared_ptr<Action> StochasticDecisionRule::operator()(const std::shared_ptr<State> &s)
    // {
    //     // return this->at(s);
    //     throw sdm::exception::NotImplementedException();
    // }

    RecursiveMap<std::shared_ptr<Action>, double> StochasticDecisionRule::getProbabilities(const std::shared_ptr<State> &state) const
    {
        try
        {
            return this->at(state);
        }
        catch(const std::exception& e)
        {
            std::cerr <<"State not found in the StochasticDecisionRule::getProbabilities"<< e.what() << '\n';
            exit(-1) ;
        }
    }

    double StochasticDecisionRule::getProbability(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action) const
    {
        try
        {
            return this->getProbabilities(state).at(action);
        }
        catch(const std::exception& e)
        {
            std::cerr <<"Action not found in the StochasticDecisionRule::getProbability"<< e.what() << '\n';
            exit(-1);
        }
    }

    void StochasticDecisionRule::setProbability(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, double proba)
    {
        (*this)[state][action] = proba;
    }

    std::string StochasticDecisionRule::str() const
    {
        std::ostringstream res;
        res << "<decision-rule type=\"stochastic\">" << std::endl;
        for (const auto &pair_state__pair_action__proba : *this)
        {
            res << "\t<decision state=\"" << pair_state__pair_action__proba.first->str() << "\">" << std::endl;

            for(const auto &pair_action_proba : pair_state__pair_action__proba.second)
            {
                res << "\t < action=\"" << pair_action_proba.first->str()<<"\" probability=\"" <<this->getProbability(pair_state__pair_action__proba.first,pair_action_proba.first)<<"/>"  << std::endl;
            }

            res << "\t<decision/>" << std::endl;
        }
        res << "<decision-rule/>" << std::endl;
        return res.str();
    }

} // namespace sdm

namespace std
{
    // template <>
    // struct hash<sdm::StochasticDecisionRule>
    // {
    //     typedef sdm::StochasticDecisionRule argument_type;
    //     typedef std::size_t result_type;
    //     inline result_type operator()(const argument_type &in) const
    //     {
    //         size_t seed = 0;
    //         for (auto &input : in)
    //         {
    //             sdm::hash_combine(seed, input.first);
    //             sdm::hash_combine(seed, input.second);
    //         }
    //         return seed;
    //     }
    // };
}
#include <sdm/core/dynamics/tabular_state_dynamics.hpp>

namespace sdm
{
    TabularStateDynamics::TabularStateDynamics()
    {
    }

    TabularStateDynamics::TabularStateDynamics(const TabularStateDynamics &copy) : t_model(copy.t_model)
    {
    }

    TabularStateDynamics::~TabularStateDynamics()
    {
    }

    void TabularStateDynamics::setReachablesStates(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, number)
    {
        this->successor_states[state][action].insert(next_state);
    }

    void TabularStateDynamics::updateNextStateDistribution(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, double proba)
    {
        // If the state is the map of state to next state distributions, we initialize this
        if (this->next_states_distrib.find(state) == this->next_states_distrib.end())
        {
            this->next_states_distrib.emplace(state, std::unordered_map<std::shared_ptr<Action>, std::shared_ptr<DiscreteDistribution<std::shared_ptr<State>>>>());
        }
        // If the distribution does not already exists
        if (this->next_states_distrib[state].find(action) == this->next_states_distrib[state].end())
        {
            this->next_states_distrib[state].emplace(action, std::make_shared<DiscreteDistribution<std::shared_ptr<State>>>());
        }
        this->next_states_distrib[state][action]->setProbability(next_state, proba);
    }

    void TabularStateDynamics::setTransitionProbability(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, double prob, number, bool cumul)
    {
            // Set reachable state
            this->setReachablesStates(state, action, next_state);
            if (cumul)
            {
                double cumul_proba = this->t_model[action].getValueAt(state, next_state) + prob;
                this->t_model[action].setValueAt(state, next_state, cumul_proba);
                this->updateNextStateDistribution(state, action, next_state, cumul_proba);
            }
            else
            {
                this->t_model[action].setValueAt(state, next_state, prob);
                this->updateNextStateDistribution(state, action, next_state, prob);
            }
    }

    double TabularStateDynamics::getTransitionProbability(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, number) const
    {
        const auto &iterator = this->t_model.find(action);
        return (iterator == this->t_model.end()) ? 0. : this->t_model.at(action).getValueAt(state, next_state);
    }

    std::set<std::shared_ptr<State>> TabularStateDynamics::getReachableStates(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number) const
    {
        try
        {
            return this->successor_states.at(state).at(action);
        }
        catch (std::exception e)
        {
            return {};
        }

        // const auto &iterator = this->successor_states.find(state);
        // if (iterator != this->successor_states.end())
        // {
        //     const auto &iterator2 = iterator->second.find(action);
        //     if (iterator2 != iterator->second.end())
        //     {
        //         return this->successor_states.at(state).at(action);
        //     }
        // }
        // return {};
    }

    std::shared_ptr<Distribution<std::shared_ptr<State>>> TabularStateDynamics::getNextStateDistribution(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t) const
    {
        return this->next_states_distrib.at(state).at(action);
        // auto next_state_distribution = std::make_shared<DiscreteDistribution<std::shared_ptr<State>>>();
        // for (const auto &reachable_state : this->getReachableStates(state, action, t))
        // {
            // next_state_distribution->setProbability(reachable_state, this->getTransitionProbability(state, action, reachable_state, t));
        // }
        // return next_state_distribution;
    }

} // namespace sdm

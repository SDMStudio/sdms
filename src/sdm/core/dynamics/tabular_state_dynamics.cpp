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

    void TabularStateDynamics::setReachablesStates(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, number )
    {
        this->successor_states[state][action].insert(next_state);
    }
    
    void TabularStateDynamics::setTransitionProbability(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, double prob, number, bool cumul)
    {
        if (prob <= 0)
        {
            this->successor_states[state][action].erase(next_state);
            this->t_model[action].setValueAt(state, next_state, prob);
        }
        else
        {
            this->setReachablesStates(state, action, next_state);
            if (cumul)
                this->t_model[action].setValueAt(state, next_state, this->t_model[action].getValueAt(state, next_state) + prob);
            else
                this->t_model[action].setValueAt(state, next_state, prob);
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

    std::shared_ptr<Distribution<std::shared_ptr<State>>> TabularStateDynamics::getNextStateDistribution(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action)
    {
        // std::set<std::shared_ptr<State>> reachable_states = this->getReachableStates(state, action);
        // std::shared_ptr<DiscreteDistribution<std::shared_ptr<State>>> next_state_distribution;

        // for (std::shared_ptr<State> reachable_state : reachable_states)
        // {
        //     next_state_distribution->setProbability(reachable_state, this->getTransitionProbability(state, action, reachable_state));
        // }
        // return std::static_pointer_cast<Distribution<std::shared_ptr<State>>>(next_state_distribution);
    }

    // void TabularStateDynamics::setTransitions(const std::unordered_map<std::shared_ptr<Action>, matrix_type> &t_model)
    // {
    //     this->t_model = t_model;
    // }

    // std::unordered_map<std::shared_ptr<Action>, TabularStateDynamics::matrix_type> TabularStateDynamics::getTransitions()
    // {
    //     return this->t_model;
    // }

    // const TabularStateDynamics::matrix_type &TabularStateDynamics::getTransitions(const std::shared_ptr<Action> &action)
    // {
    //     return this->t_model[action];
    // }
} // namespace sdm

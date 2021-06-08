#include <sdm/core/dynamics/tabular_state_dynamics.hpp>

namespace sdm
{
    TabularStateDynamics::TabularStateDynamics()
    {
    }

    TabularStateDynamics::TabularStateDynamics(const TabularStateDynamics &copy) : t_model(copy.t_model)
    {
    }

    // TabularStateDynamics::TabularStateDynamics(number num_actions, number num_states)
    // {
    //     this->initDynamics(num_actions, num_states);
    // }

    TabularStateDynamics::~TabularStateDynamics()
    {
    }

    void TabularStateDynamics::initDynamics(number, number)
    {
        // for (number a = 0; a < num_actions; ++a)
        // {
        //     this->t_model.push_back(matrix_type(num_states, num_states));
        // }
    }

    void TabularStateDynamics::setReachablesStates(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, number )
    {
        this->successor_states[state][action].insert(next_state);
    }

    void TabularStateDynamics::setTransitionProbability(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, double prob, number, bool cumul)
    {
        if (prob > 0)
        {
            this->setReachablesStates(state,action,next_state);
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

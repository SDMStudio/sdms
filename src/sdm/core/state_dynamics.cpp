/*=============================================================================
Copyright (C) 2016 Jilles Steeve Dibangoye
==============================================================================*/
#include <boost/numeric/ublas/matrix_sparse.hpp>
#include <boost/numeric/ublas/vector.hpp>

#include <sdm/core/state_dynamics.hpp>
#include <sdm/utils/linear_algebra/matrix.hpp>

namespace sdm
{
    StateDynamics::StateDynamics()
    {
    }

    StateDynamics::StateDynamics(StateDynamics &copy) : t_model(copy.getTransitions())
    {
    }

    StateDynamics::StateDynamics(number num_actions, number num_states)
    {
        this->initDynamics(num_actions, num_states);
    }

    void StateDynamics::initDynamics(number num_actions, number num_states)
    {
        for (number a = 0; a < num_actions; ++a)
        {
            this->t_model.push_back(Matrix(num_states, num_states));
        }
    }

    void StateDynamics::setTransitionProbability(number x, number u, number y, double prob, bool cumul)
    {
        if (cumul)
            this->t_model[u](x, y) += prob;
        else
            this->t_model[u](x, y) = prob;
    }

    double StateDynamics::getTransitionProbability(number x, number u, number y) const
    {
        return this->t_model[u](x, y);
    }

    const std::unordered_set<state> &StateDynamics::getStateSuccessors(number x, number u)
    {
        return this->successor_states.at(x).at(u);
    }

    void StateDynamics::setTransitions(const std::vector<Matrix> &t_model)
    {
        this->t_model = t_model;
    }

    std::vector<Matrix> StateDynamics::getTransitions()
    {
        return this->t_model;
    }

    const Matrix &StateDynamics::getTransitions(number u)
    {
        return this->t_model[u];
    }
} // namespace sdm

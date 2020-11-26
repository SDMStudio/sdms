/*=============================================================================
Copyright (C) 2016 Jilles Steeve Dibangoye
==============================================================================*/
#include <boost/numeric/ublas/matrix_sparse.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>

#include <sdm/core/state_dynamics.hpp>
#include <sdm/utils/linear_algebra/matrix.hpp>

namespace sdm
{
    StateDynamics::StateDynamics()
    {
    }

    StateDynamics::StateDynamics(number num_jactions, number num_states)
    {
        this->initDynamics(num_jactions, num_states);
    }

    void StateDynamics::initDynamics(number num_jactions, number num_states)
    {
        number a;

        for (a = 0; a < num_jactions; ++a)
        {
            this->t_model.push_back(Matrix(num_states, num_states));
        }
    }

    void StateDynamics::setTransitionProbability(number x, number jaction, number y, double prob, bool cumul)
    {
        if (cumul)
            this->t_model[jaction](x, y) += prob;
        else
            this->t_model[jaction](x, y) = prob;
    }

    double StateDynamics::getTransitionProbability(number x, number jaction, number y) const
    {
        return this->t_model[jaction](x, y);
    }

    const std::unordered_set<state> &StateDynamics::getStateSuccessors(number x, number jaction)
    {
        return this->successor_states.at(x).at(jaction);
    }

    void StateDynamics::setTransitions(const std::vector<Matrix> &t_model)
    {
        this->t_model = t_model;
    }

    const Matrix &StateDynamics::getTransitions(number u)
    {
        return this->t_model[u];
    }
} // namespace sdm

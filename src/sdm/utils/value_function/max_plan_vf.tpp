#include <sdm/utils/value_function/max_plan_vf.hpp>

namespace sdm
{

    template <typename TVector, typename TAction, typename TValue>
    MaxPlanValueFunction<TVector, TAction, TValue>::MaxPlanValueFunction() {}

    template <typename TVector, typename TAction, typename TValue>
    MaxPlanValueFunction<TVector, TAction, TValue>::MaxPlanValueFunction(std::shared_ptr<SolvableByHSVI<TVector, TAction>> problem, int horizon, std::shared_ptr<Initializer<TVector, TAction>> initializer)
        : ValueFunction<TVector, TAction, TValue>(problem, horizon), initializer_(initializer)
    {
        this->representation = std::vector<HyperplanSet>(this->isInfiniteHorizon() ? 1 : this->horizon_, HyperplanSet({}));
        this->initialize();
    }

    template <typename TVector, typename TAction, typename TValue>
    MaxPlanValueFunction<TVector, TAction, TValue>::MaxPlanValueFunction(std::shared_ptr<SolvableByHSVI<TVector, TAction>> problem, int horizon, TValue default_value) : MaxPlanValueFunction(problem, horizon, std::make_shared<ValueInitializer<TVector, TAction>>(default_value))
    {
    }

    template <typename TVector, typename TAction, typename TValue>
    void MaxPlanValueFunction<TVector, TAction, TValue>::initialize(TValue value, int t)
    {
        TVector new_v(value);
        this->representation[t].emplace(new_v);
    }

    template <typename TVector, typename TAction, typename TValue>
    void MaxPlanValueFunction<TVector, TAction, TValue>::initialize()
    {
        this->initializer_->init(this);
    }

    template <typename TVector, typename TAction, typename TValue>
    std::pair<TValue, TVector> MaxPlanValueFunction<TVector, TAction, TValue>::getMaxAt(const TVector &state, int t)
    {
        TValue current, max = std::numeric_limits<TValue>::min();
        TVector alpha_vector;
        for (const auto &plan : this->representation[t])
        {
            current = plan ^ state;
            if (max < current)
            {
                max = current;
                alpha_vector = plan;
            }
        }
        return {max, alpha_vector};
    }

    template <typename TVector, typename TAction, typename TValue>
    TValue MaxPlanValueFunction<TVector, TAction, TValue>::getValueAt(const TVector &state, int t)
    {
        return this->getMaxAt(state, t).first;
    }

    template <typename TVector, typename TAction, typename TValue>
    TValue MaxPlanValueFunction<TVector, TAction, TValue>::operator()(const TVector &state)
    {
        return this->getValueAt(state);
    }

    template <typename TVector, typename TAction, typename TValue>
    TValue MaxPlanValueFunction<TVector, TAction, TValue>::getQValueAt(const TVector &state, const TAction &action, int t)
    {
        // implement bellman operator
        return this->getWorld()->getReward(state, action) + this->getWorld()->getDiscount() * this->getWorld()->getExpectedNextValue(this, state, action, t);
    }

    template <typename TVector, typename TAction, typename TValue>
    std::shared_ptr<VectorImpl<TAction, TValue>> MaxPlanValueFunction<TVector, TAction, TValue>::getQValueAt(const TVector &state, int t)
    {
        std::shared_ptr<MappedVector<TAction, double>> q_s = std::make_shared<MappedVector<TAction, double>>();
        for (auto &a : this->getWorld()->getActionSpace(state).getAll())
        {
            (*q_s)[a] = this->getQValueAt(state, a, t);
        }
        return q_s;
    }

    template <typename TVector, typename TAction, typename TValue>
    TAction MaxPlanValueFunction<TVector, TAction, TValue>::getBestAction(const TVector &state, int t)
    {
        auto qvalues = this->getQValueAt(state, t);
        return qvalues->argmax();
    }

    template <typename TVector, typename TAction, typename TValue>
    void MaxPlanValueFunction<TVector, TAction, TValue>::updateValueAt(const TVector &state, int t)
    {
        // ERROR : To change with true bellman backup ope
        TVector new_hyperplan = state;

        if (this->isInfiniteHorizon())
        {
            this->representation[0].emplace(new_hyperplan);
        }
        else
        {
            assert(t < this->horizon_);
            this->representation[t].emplace(new_hyperplan);
        }
        this->prune(t);
    }

    template <typename TVector, typename TAction, typename TValue>
    number MaxPlanValueFunction<TVector, TAction, TValue>::size()
    {
        return this->representation.size();
    }

    template <typename TVector, typename TAction, typename TValue>
    void MaxPlanValueFunction<TVector, TAction, TValue>::prune(int t)
    {
        this->bounded_prune(t);
    }

    template <typename TVector, typename TAction, typename TValue>
    void MaxPlanValueFunction<TVector, TAction, TValue>::bounded_prune(int t)
    {
        std::map<TVector, number> refCount;

        //<! initialize the count for each hyperplan
        for (const auto &plan : (this->isInfiniteHorizon() ? this->representation[0] : this->representation[t]))
        {
            refCount.emplace(plan, 0);
        }

        //<! update the count
        TVector max_alpha;
        TValue max_value = std::numeric_limits<TValue>::min(), value;
        for (const auto &frequency : (this->isInfiniteHorizon() ? this->representation[0] : this->representation[t]))
        {
            for (const auto &alpha : refCount)
            {
                if (max_value < (value = (alpha.first) ^ (frequency)))
                {
                    max_value = value;
                    max_alpha = alpha.first;
                }
            }

            if (refCount.find(max_alpha) != refCount.end())
            {
                refCount.at(max_alpha)++;
            }
        }

        //<! remove dominated alpha-vectors
        if (this->isInfiniteHorizon())
        {
            for (const auto &alpha : this->representation[0])
            {
                if (refCount.at(alpha) == 0)
                    this->representation[0].erase(alpha);
            }
        }
        else
        {
            assert(t < this->horizon_);
            for (const auto &alpha : this->representation[t])
            {
                if (refCount.at(alpha) == 0)
                    this->representation[t].erase(alpha);
            }
        }
    }

    // template <typename TVector, typename TAction, typename TValue>
    // TVector MaxPlanValueFunction<TVector, TAction, TValue>::backup_bellman_operator(std::shared_ptr<SolvableByHSVI<TVector, TAction>> world, TVector state, int t)
    // {
    //     std::vector<std::vector<TVector>> beta_a_o(world->getNumObservations(), std::vector<TVector>(world->getNumActions(), TVector(world->getNumStates()));
    //     std::vector<TVector> beta_a(world->getNumActions(), TVector(world->getNumStates());

    //     // beta_a_o = argmax_alpha ( alpha * belief_t+1)
    //     for (number a=0; a < world->getNumActions(); a++)
    //     {
    //         for (number o = 0; o < world->getNumObservations(); o++)
    //         {
    //             beta_a_o[a][o] = this->getMaxAt(world->getNextState(state, a, o)).second;
    //         }
    //     }
    //     // \beta_a = R(s,a) + \gamma * \sum_{o, s'} [ \beta_{a,o}(s') * O(s', a, o) * T(s,a,s') ]
    //     for (number a = 0; a < world->getNumActions(); a++)
    //     {
    //         for (number s = 0; s < world->getNumStates(); s++)
    //         {
    //             double tmp = 0;
    //             for (number o = 0; o < world->getNumObservations(); o++)
    //             {
    //                 for (number s_ = 0; s_ < world->getNumStates(); s_++)
    //                 {
    //                     tmp += beta_a_o[a][o](s_) * world->getDynamics(s, a, o, s_);
    //                 }
    //             }
    //             beta_a[a](s) = world->getReward(s, a) + world->getDiscount() * tmp;
    //         }
    //     }
    //     number a_max;
    //     double current, max_v = std::numeric_limits<double>::min();
    //     for (number a = 0; a < world->getNumActions(); a++)
    //     {
    //         current = beta_a[a] ^ state;
    //         if (current > max_v)
    //         {
    //             max_v = current;
    //             a_max = a;
    //         }
    //     }
    //     return beta_a[a_max];
    // }

} // namespace sdm
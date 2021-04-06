#include <sdm/utils/value_function/value_function.hpp>

namespace sdm
{

    template <typename TVector, typename TAction, typename TValue>
    MaxPlanValueFunction<TVector, TAction, TValue>::MaxPlanValueFunction() {}

    template <typename TVector, typename TAction, typename TValue>
    MaxPlanValueFunction<TVector, TAction, TValue>::MaxPlanValueFunction(std::shared_ptr<SolvableByHSVI<TVector, TAction>> problem, int horizon, std::shared_ptr<Initializer<TVector, TAction>> initializer)
        : ValueFunction<TVector, TAction, TValue>(problem, horizon), initializer_(initializer)
    {
        this->representation = std::vector<HyperplanSet>(this->isInfiniteHorizon() ? 1 : this->horizon_, HyperplanSet({}));
    }

    template <typename TVector, typename TAction, typename TValue>
    MaxPlanValueFunction<TVector, TAction, TValue>::MaxPlanValueFunction(std::shared_ptr<SolvableByHSVI<TVector, TAction>> problem, int horizon, TValue default_value) : MaxPlanValueFunction(problem, horizon, std::make_shared<ValueInitializer<TVector, TAction>>(default_value))
    {
    }

    template <typename TVector, typename TAction, typename TValue>
    void MaxPlanValueFunction<TVector, TAction, TValue>::initialize(TValue value, number t)
    {
        TVector new_v(value);
        if (this->isInfiniteHorizon())
        {
            this->representation[0].push_back(new_v);
        }
        else
        {
            assert(t < this->getHorizon());
            this->representation[t].push_back(new_v);
        }
    }

    template <typename TVector, typename TAction, typename TValue>
    void MaxPlanValueFunction<TVector, TAction, TValue>::initialize()
    {
        this->initializer_->init(this);
    }

    template <typename TVector, typename TAction, typename TValue>
    std::pair<TValue, TVector> MaxPlanValueFunction<TVector, TAction, TValue>::getMaxAt(const TVector &state, number t)
    {
        TValue current, max = -std::numeric_limits<TValue>::max();
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
    TValue MaxPlanValueFunction<TVector, TAction, TValue>::getValueAt(const TVector &state, number t)
    {
        if (this->isInfiniteHorizon())
        {
            return this->getMaxAt(state, 0).first;
        }
        else
        {
            return (t >= this->getHorizon()) ? 0 : this->getMaxAt(state, t).first;
        }
    }

    template <typename TVector, typename TAction, typename TValue>
    void MaxPlanValueFunction<TVector, TAction, TValue>::updateValueAt(const TVector &state, number t)
    {
        // TODO : To change with true bellman backup ope
        auto new_hyperplan = this->backup_operator<TVector>(state, t);

        std::cout << new_hyperplan << std::endl;
        if (this->isInfiniteHorizon())
        {
            this->representation[0].push_back(new_hyperplan);
        }
        else
        {
            assert(t < this->horizon_);
            this->representation[t].push_back(new_hyperplan);
        }
        // this->prune(t);
    }

    template <typename TVector, typename TAction, typename TValue>
    std::vector<TVector> MaxPlanValueFunction<TVector, TAction, TValue>::getSupport(number t)
    {
        std::vector<TVector> output;
        if (this->isInfiniteHorizon())
        {
            output = std::vector<TVector>(this->representation[0].begin(), this->representation[0].end());
            return output;
        }
        else
        {
            output = std::vector<TVector>(this->representation[t].begin(), this->representation[t].end());
            return (t >= this->getHorizon()) ? std::vector<TVector>{} : output;
        }
    }

    template <typename TVector, typename TAction, typename TValue>
    void MaxPlanValueFunction<TVector, TAction, TValue>::prune(number t)
    {
        this->bounded_prune(t);
    }

    template <typename TVector, typename TAction, typename TValue>
    void MaxPlanValueFunction<TVector, TAction, TValue>::bounded_prune(number t)
    {
        std::map<TVector, number> refCount;

        //<! initialize the count for each hyperplan
        for (const auto &plan : (this->isInfiniteHorizon() ? this->representation[0] : this->representation[t]))
        {
            refCount.emplace(plan, 0);
        }

        //<! update the count
        TVector max_alpha;
        TValue max_value = -std::numeric_limits<TValue>::max(), value;
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

    template <typename TVector, typename TAction, typename TValue>
    number MaxPlanValueFunction<TVector, TAction, TValue>::size()
    {
        return this->representation.size();
    }

    // For OccupancyMDP (i.e. OccupancyState as vector type)
    template <typename TVector, typename TAction, typename TValue>
    template <typename T, std::enable_if_t<std::is_same_v<OccupancyState<>, T>, int>>
    TVector MaxPlanValueFunction<TVector, TAction, TValue>::backup_operator(const TVector &state, number t)
    {
        std::cout << "Formalism DecPOMDP" << std::endl;
        // auto oMDP = std::static_pointer_cast<OccupancyMDP<>>(this->getWorld());
        // auto under_pb = this->getWorld()->getUnderlyingProblem();

        // TVector v_max;
        // double value_max = -std::numeric_limits<double>::max(), tmp;
        // // Parcours des hyperplan support de la fonction au pas t+1
        // for (const auto &plan : this->representation[t + 1])
        // {
        //     // Boucle over all joint decision rule at occupancy state
        //     for (const auto &jdr : oMDP->getActionSpaceAt(state)->getAll())
        //     {
        //         TVector v;
        //         for (const auto &pair_s_o_p : state)
        //         {
        //             auto pair_s_o = pair_s_o_p.first;
        //             auto joint_history = pair_s_o.second;
        //             auto proba = pair_s_o_p.second;

        //             // Get joint action from JointDetDecisionRule
        //             std::vector<typename oAction::value_type::output_type> jaction;
        //             for (int i = 0; i < jdr.size(); i++)
        //             {
        //                 auto p_ihist = joint_history->getIndividualHistory(i);
        //                 auto idr = jdr.at(i);
        //                 jaction.push_back(idr(p_ihist));
        //             }

        //             //
        //             v[pair_s_o] = under_pb->getReward(pair_s_o.first, under_pb->getActionSpace()->joint2single(jaction));
        //             for (auto &o : under_pb->getObsSpace()->getAll())
        //             {
        //                 auto joint_history_next = joint_history->expand(o);
        //                 for (number s_ = 0; s_ < under_pb->getStateSpace()->getNumItems())
        //                 {
        //                     v[pair_s_o] += under_pb->getDiscount() * under_pb->getObsDynamics()->getDynamics(pair_s_o.first, under_pb->getActionSpace()->joint2single(jaction), o, s_) * plan[Pair(s_, joint_history_next)];
        //                 }
        //             }
        //         }

        //         if (value_max < (tmp = v ^ state))
        //         {
        //             value_max = tmp;
        //             v_max = v;
        //         }
        //     }
        // }

        // return v_max;
        throw sdm::exception::NotImplementedException();
    }

    // For SerializedOccupancyMDP (i.e. SerializedOccupancyState as vector type)
    template <typename TVector, typename TAction, typename TValue>
    template <typename T, std::enable_if_t<std::is_same_v<SerializedOccupancyState<>, T>, int>>
    TVector MaxPlanValueFunction<TVector, TAction, TValue>::backup_operator(const TVector &state, number t)
    {
        std::cout << "Formalism SerializedDecPOMDP" << std::endl;
        throw sdm::exception::NotImplementedException();
    }

    // For BeliefMDP
    template <typename TVector, typename TAction, typename TValue>
    template <typename T, std::enable_if_t<std::is_same_v<BeliefState, T>, int>>
    TVector MaxPlanValueFunction<TVector, TAction, TValue>::backup_operator(const TVector &state, number t)
    {
        auto beliefMDP = std::static_pointer_cast<BeliefMDP<>>(this->getWorld());
        auto under_pb = this->getWorld()->getUnderlyingProblem();

        number n_obs = under_pb->getObsSpace()->getNumItems();
        number n_actions = under_pb->getActionSpace()->getNumItems();
        number n_states = under_pb->getStateSpace()->getNumItems();

        std::vector<std::vector<TVector>> beta_a_o(n_actions, std::vector<TVector>(n_obs, TVector()));
        std::vector<TVector> beta_a(n_actions, TVector());

        // beta_a_o = argmax_alpha ( alpha * belief_t+1)
        for (number a = 0; a < n_actions; a++)
        {
            for (number o = 0; o < n_obs; o++)
            {
                auto next_belief = beliefMDP->nextState(state, a, o);
                beta_a_o[a][o] = this->getMaxAt(next_belief, t).second;
            }
        }
        // \beta_a = R(s,a) + \gamma * \sum_{o, s'} [ \beta_{a,o}(s') * O(s', a, o) * T(s,a,s') ]
        for (number a = 0; a < n_actions; a++)
        {
            for (number s = 0; s < n_states; s++)
            {
                double tmp = 0;
                for (number o = 0; o < n_obs; o++)
                {
                    for (number s_ = 0; s_ < n_states; s_++)
                    {
                        tmp += beta_a_o[a][o].at(s_) * under_pb->getObsDynamics()->getDynamics(s, a, o, s_);
                    }
                }
                beta_a[a][s] = under_pb->getReward(s, a) + under_pb->getDiscount() * tmp;
            }
        }
        number a_max;
        double current, max_v = -std::numeric_limits<double>::max();
        for (number a = 0; a < n_actions; a++)
        {
            current = beta_a[a] ^ state;
            if (current > max_v)
            {
                max_v = current;
                a_max = a;
            }
        }
        auto new_plan = beta_a[a_max];
        return new_plan;
    }

} // namespace sdm
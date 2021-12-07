#include <sdm/core/state/interface/belief_interface.hpp>
#include <sdm/core/state/interface/occupancy_state_interface.hpp>
#include <sdm/world/base/pomdp_interface.hpp>
#include <sdm/world/base/belief_mdp_interface.hpp>

namespace sdm
{
    template <class Hash, class KeyEqual>
    BaseSawtoothValueFunction<Hash, KeyEqual>::BaseSawtoothValueFunction(const std::shared_ptr<SolvableByDP> &world,
                                                                         const std::shared_ptr<Initializer> &initializer,
                                                                         const std::shared_ptr<ActionSelectionInterface> &action_selection,
                                                                         const std::shared_ptr<TabularUpdateOperator> &update_operator,
                                                                         int freq_pruning,
                                                                         SawtoothPruning::Type type_of_sawtooth_prunning)
        : ValueFunctionInterface(world, initializer, action_selection),
          BaseTabularValueFunction<Hash, KeyEqual>(world, initializer, action_selection, update_operator),
          PrunableStructure(world->getHorizon(), freq_pruning),
          type_of_sawtooth_prunning_(type_of_sawtooth_prunning)
    {
        this->relaxation = std::vector<Container>(this->isInfiniteHorizon() ? 1 : this->horizon_ + 1, Container());
    }

    template <class Hash, class KeyEqual>
    BaseSawtoothValueFunction<Hash, KeyEqual>::BaseSawtoothValueFunction(const BaseSawtoothValueFunction &copy)
        : ValueFunctionInterface(copy.world_, copy.initializer_, copy.action_selection_),
          BaseTabularValueFunction<Hash, KeyEqual>(copy),
          PrunableStructure(copy.world_->getHorizon(), copy.freq_pruning),
          type_of_sawtooth_prunning_(copy.type_of_sawtooth_prunning_)
    {
    }

    template <class Hash, class KeyEqual>
    double BaseSawtoothValueFunction<Hash, KeyEqual>::getValueAt(const std::shared_ptr<State> &state, number t)
    {
        return this->evaluate(state, t).second;
    }

    template <class Hash, class KeyEqual>
    void BaseSawtoothValueFunction<Hash, KeyEqual>::setValueAt(const std::shared_ptr<State> &state, double new_value, number t)
    {
        // assert((getValueAt(state, t) >= new_value) && "New value is higher than the old");
        BaseTabularValueFunction<Hash, KeyEqual>::setValueAt(state, new_value, t);
    }

    template <class Hash, class KeyEqual>
    double BaseSawtoothValueFunction<Hash, KeyEqual>::getRelaxedValueAt(const std::shared_ptr<State> &state, number t)
    {
        double relaxed_value = 0;
        auto iter_relax = this->relaxation[t].find(state);
        if (iter_relax == this->relaxation[t].end())
        {
            if (this->getInitFunction())
                relaxed_value = this->getInitFunction()->operator()(state, t);
            else
                relaxed_value = this->representation[t].getDefault();
            this->relaxation[t].emplace(state, relaxed_value);
        }
        else
        {
            relaxed_value = iter_relax->second;
        }
        return relaxed_value;
    }

    template <class Hash, class KeyEqual>
    Pair<std::shared_ptr<State>, double> BaseSawtoothValueFunction<Hash, KeyEqual>::evaluate(const std::shared_ptr<State> &state, number t)
    {

        auto iterator_on_point = this->representation[t].find(state);
        if (iterator_on_point != this->representation[t].end())
        {
            return *iterator_on_point;
        }
        else
        {
            // Compute v(s) = v^{relax}(s) + min_k min_{x\in Supp(s^k)} \frac{s(x)}{s^k(x)} \left( v^{relax}(s^k) - v^k\right)
            std::shared_ptr<State> argmin_k = state;
            double min_k = (this->getSupport(t).size() == 0) ? 0.0 : std::numeric_limits<double>::max();
            double v_relax = this->getRelaxedValueAt(state, t);

            // Go over all points in the representation
            for (const auto &point_k : this->representation[t])
            {
                // Dissociate element of the k-th point (state / value)
                auto [s_k, v_k] = point_k;

                // Determine the "value" for k-th point
                double min_int = this->computeRatio(state, s_k) * (v_k - this->getRelaxedValueAt(s_k, t));

                // If the "value" of k-th point is minimal, keep it
                if (min_int < min_k)
                {
                    min_k = min_int;
                    argmin_k = s_k;
                }
            }
            return std::make_pair(argmin_k, v_relax + min_k);
        }
    }

    template <class Hash, class KeyEqual>
    double BaseSawtoothValueFunction<Hash, KeyEqual>::getSawtoothValueAt(const std::shared_ptr<OccupancyStateInterface> &s, const std::shared_ptr<JointHistoryInterface> o,
                                                                         const std::shared_ptr<Action> &u, const std::shared_ptr<OccupancyStateInterface> &s_k,
                                                                         const std::shared_ptr<State> &y, const std::shared_ptr<JointHistoryInterface> &o_,
                                                                         const std::shared_ptr<Observation> &z, number t, bool display)
    {
        // Compute the value : Q^{relax}(b,u) +  p(b') * b'(x') / s^{k}(x', o')] (v^{k} - v^{relax}(s^{k}))
        // --> Q_relax + min_ratio * (v_k - v_relax)
        // This value will be multiplied by p(o) to produce the coefficient of the corresponding decision rule for the LP

        auto oMDP = std::dynamic_pointer_cast<BeliefMDPInterface>(this->getWorld());
        auto relaxation = std::static_pointer_cast<RelaxedValueFunction>(this->getInitFunction());
        auto b = s->getBeliefAt(o);

        double coef = 0.;
        if (o->expand(z) == o_)
        {

            auto [next_belief, proba_next_b] = oMDP->getUnderlyingBeliefMDP()->getNextStateAndProba(b, u, z, t);
            auto next_b = next_belief->toBelief();
            coef = (getValueAt(s_k, t + 1) - getRelaxedValueAt(s_k, t + 1)) * (proba_next_b * next_b->getProbability(y) / s_k->toOccupancyState()->getProbability(o_, y));
        }

        return s->getProbability(o) * (relaxation->getQValueAt(b, u, t) + coef);

        // // Compute the first term : Q_relax = Q^{relax} (b,u)
        // double Q_relax = relaxation->getQValueAt(b, u, t);
        // double v_k = 0., v_relax = 0., min_ratio = 0., tmp;

        // if (o->expand(z) == o_)
        // {

        //     // Compute the next belief
        //     auto [next_belief, proba_next_b] = oMDP->getUnderlyingBeliefMDP()->getNextStateAndProba(b, u, z, t);
        //     auto next_b = next_belief->toBelief();
        //     if (display)
        //     {
        //         std::cout << "--- next_b=" << next_b->str() << " - proba=" << proba_next_b << std::endl;
        //     }

        //     // Compute the min ratio : min_{x'} [ p(b') * b'(x') / s^{(k)}(x', o')]
        //     min_ratio = std::numeric_limits<double>::max();
        //     for (auto &y : s_k->getBeliefAt(o_)->getStates())
        //     {
        //         tmp = proba_next_b * next_b->getProbability(y) / s_k->getProbability(o_, y);

        //         if (tmp < min_ratio)
        //         {
        //             min_ratio = tmp;
        //         }
        //     }
        //     v_k = this->getValueAt(s_k, t + 1);            // Get v^k
        //     v_relax = this->getRelaxedValueAt(s_k, t + 1); // Get v^relax(s^k)
        // }
        // if (display)
        // {
        //     std::cout << "--- o=" << o->short_str() << " - u=" << u->str() << std::endl;
        //     std::cout << "#> [" << Q_relax << " + " << min_ratio << " * (" << v_k << " - " << v_relax << ")] * ";
        // }
        // return s->getProbability(o) * (Q_relax + min_ratio * (v_k - v_relax));
    }

    template <class Hash, class KeyEqual>
    double BaseSawtoothValueFunction<Hash, KeyEqual>::computeRatio(const std::shared_ptr<State> &s, const std::shared_ptr<State> &s_k)
    {
        // Check available ratio in the map and return it
        auto iter_s = ratios.find(s);
        if (iter_s != ratios.end())
        {
            auto iter_s_k = iter_s->second.find(s_k);
            if (iter_s_k != iter_s->second.end())
            {
                return iter_s_k->second;
            }
        }
        double ratio = 0.;
        // If no such ratio is already stored, compute it and store it
        if (sdm::isInstanceOf<OccupancyStateInterface>(s))
        {
            ratio = this->ratioOccupancy(sdm::to<OccupancyStateInterface>(s), sdm::to<OccupancyStateInterface>(s_k));
        }
        else if (sdm::isInstanceOf<BeliefInterface>(s))
        {
            ratio = this->ratioBelief(sdm::to<BeliefInterface>(s), sdm::to<BeliefInterface>(s_k));
        }
        else
        {
            throw sdm::exception::Exception("(PointSet::computeRatio) States must inherit from 'BeliefInterface' or 'OccupancyStateInterface'");
        }
        ratios[s][s_k] = ratio;
        return ratio;
    }

    template <class Hash, class KeyEqual>
    double BaseSawtoothValueFunction<Hash, KeyEqual>::ratioBelief(const std::shared_ptr<BeliefInterface> &b, const std::shared_ptr<BeliefInterface> &b_k)
    {
        // Determine the ratio for the specific case when the state is a belief
        double min_ratio = 1.0;

        for (auto &x : b_k->getStates())
        {
            double ratio_k_x = (b->getProbability(x) / b_k->getProbability(x));
            // determine the min ratio
            if (ratio_k_x < min_ratio)
            {
                min_ratio = ratio_k_x;
            }
        }
        return min_ratio;
    }

    template <class Hash, class KeyEqual>
    double BaseSawtoothValueFunction<Hash, KeyEqual>::ratioOccupancy(const std::shared_ptr<OccupancyStateInterface> &s, const std::shared_ptr<OccupancyStateInterface> &s_k)
    {
        // Determine the ratio for the specific case when the state is a Occupancy State
        double min_ratio = 1.0;

        // Go over all joint history
        for (auto &o : s_k->getJointHistories())
        {
            // Go over all hidden state in the belief conditionning to the joitn history
            for (const auto &x : s_k->getBeliefAt(o)->getStates())
            {
                double ratio_k_o_x = (s->getProbability(o, x) / s_k->getProbability(o, x));
                // determine the min int
                if (ratio_k_o_x < min_ratio)
                {
                    min_ratio = ratio_k_o_x;
                }
            }
        }
        return min_ratio;
    }

    template <class Hash, class KeyEqual>
    void BaseSawtoothValueFunction<Hash, KeyEqual>::pairwise_prune(number t, double epsilon)
    {
        // List of points that are \eps-dominated
        std::vector<std::shared_ptr<State>> point_to_delete;
        // List of points that are not dominated
        std::vector<std::shared_ptr<State>> point_to_keep;

        // Go over all current points
        for (const auto &state_i : this->getSupport(t))
        {
            bool point_dominated = false;

            // Go over all points in point_to_keep
            for (auto state_j = point_to_keep.begin(); state_j != point_to_keep.end(); state_j++)
            {
                double estimation_i = this->getRelaxedValueAt(state_i, t) + this->computeRatio(state_i, *state_j) * (this->getValueAt(*state_j, t) - this->getRelaxedValueAt(*state_j, t));
                // If state_j pairwise epsilon-dominates state_i, we add state_i to the set of points to be deleted.
                if (estimation_i < this->getValueAt(state_i, t) + epsilon)
                {
                    point_to_delete.push_back(state_i);
                    point_dominated = true;
                    break;
                }
            }
            // If point is dominated, we reject alpha and go to the next iteration of outer loop
            if (point_dominated)
                continue;

            // Go over all points in point_to_keep
            for (auto state_j = point_to_keep.begin(); state_j != point_to_keep.end();)
            {
                // Compute estimation of state_i based upon that of state_j
                double estimation_j = this->getRelaxedValueAt(*state_j, t) + this->computeRatio(*state_j, state_i) * (this->getValueAt(state_i, t) - this->getRelaxedValueAt(state_i, t));

                // If state_i pairwise epsilon-dominates state_j in point_to_keep, we deleted this state
                if (estimation_j < this->getValueAt(*state_j, t) + epsilon)
                {
                    point_to_delete.push_back(*state_j);
                    state_j = point_to_keep.erase(state_j);
                }
                else
                {
                    state_j++;
                }
            }

            point_to_keep.push_back(state_i);
        }

        // Erase pairwise epsilon-dominated points
        for (const auto &to_delete : point_to_delete)
        {
            this->representation[t].erase(to_delete);
        }
    }

    template <class Hash, class KeyEqual>
    void BaseSawtoothValueFunction<Hash, KeyEqual>::prune(number t)
    {
        if (this->type_of_sawtooth_prunning_ == SawtoothPruning::PAIRWISE)
            this->pairwise_prune(t);
    }

    template <class Hash, class KeyEqual>
    std::shared_ptr<ValueFunctionInterface> BaseSawtoothValueFunction<Hash, KeyEqual>::copy()
    {
        auto casted_value = std::dynamic_pointer_cast<BaseSawtoothValueFunction<Hash, KeyEqual>>(this->getptr());
        return std::make_shared<BaseSawtoothValueFunction<Hash, KeyEqual>>(*casted_value);
    }

    template <class Hash, class KeyEqual>
    std::string BaseSawtoothValueFunction<Hash, KeyEqual>::str() const
    {
        std::ostringstream res;
        res << "<point_set_representation horizon=\"" << ((this->isInfiniteHorizon()) ? "inf" : std::to_string(this->getHorizon())) << "\">" << std::endl;
        for (std::size_t i = 0; i < this->representation.size(); i++)
        {
            res << "\t<value timestep=\"" << ((this->isInfiniteHorizon()) ? "all" : std::to_string(i)) << "\" default=\"" << this->representation.at(i).getDefault() << "\">" << std::endl;
            for (const auto &pair_st_val : this->representation.at(i))
            {
                // res << "\t\t<state id=\"" << pair_st_val.first << "\">" << std::endl;
                // res << "\t\t</state>" << std::endl;
                std::ostringstream state_str;
                state_str << pair_st_val.first /* ->str() */;
                res << "\t\t<state>" << std::endl;
                res << tools::addIndent(state_str.str(), 3) << std::endl;
                res << "\t\t</state>" << std::endl;
                res << "\t\t<value>" << std::endl;
                res << "\t\t\t" << pair_st_val.second << std::endl;
                res << "\t\t</value>" << std::endl;
            }
            res << "\t</value>" << std::endl;
        }

        res << "</point_set_representation>" << std::endl;
        return res.str();
    }

} // namespace sdm
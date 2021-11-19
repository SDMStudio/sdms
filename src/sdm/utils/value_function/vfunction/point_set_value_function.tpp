#include <sdm/core/state/interface/belief_interface.hpp>
#include <sdm/core/state/interface/occupancy_state_interface.hpp>

#include <sdm/utils/value_function/action_selection/lp/action_sawtooth_lp.hpp>
#include <sdm/utils/value_function/action_selection/lp/action_sawtooth_lp_serial.hpp>

namespace sdm
{
    template <class Hash, class KeyEqual>
    BasePointSetValueFunction<Hash, KeyEqual>::BasePointSetValueFunction(const std::shared_ptr<SolvableByDP> &world,
                                                                         const std::shared_ptr<Initializer> &initializer,
                                                                         const std::shared_ptr<ActionSelectionInterface> &action_selection,
                                                                         const std::shared_ptr<TabularUpdateOperator> &update_operator,
                                                                         int freq_pruning,
                                                                         SawtoothPrunning::Type type_of_sawtooth_prunning)
        : ValueFunctionInterface(world, initializer, action_selection),
          BaseTabularValueFunction<Hash, KeyEqual>(world, initializer, action_selection, update_operator),
          PrunableStructure(world->getHorizon(), freq_pruning),
          type_of_sawtooth_prunning_(type_of_sawtooth_prunning)
    {
#ifdef WITH_CPLEX
        if (isInstanceOf<ActionSelectionSawtoothLP>(action_selection))
        {
            this->is_sawtooth_lp = true;
        }
#endif
    }

    template <class Hash, class KeyEqual>
    BasePointSetValueFunction<Hash, KeyEqual>::BasePointSetValueFunction(const BasePointSetValueFunction &copy)
        : ValueFunctionInterface(copy.world_, copy.initializer_, copy.action_selection_),
          BaseTabularValueFunction<Hash, KeyEqual>(copy),
          PrunableStructure(copy.world_->getHorizon(), copy.freq_pruning),
          type_of_sawtooth_prunning_(copy.type_of_sawtooth_prunning_),
          is_sawtooth_lp(copy.is_sawtooth_lp)
    {
    }

    template <class Hash, class KeyEqual>
    double BasePointSetValueFunction<Hash, KeyEqual>::getValueAt(const std::shared_ptr<State> &state, number t)
    {
        return this->evaluate(state, t).second;
    }

    template <class Hash, class KeyEqual>
    double BasePointSetValueFunction<Hash, KeyEqual>::getRelaxedValueAt(const std::shared_ptr<State> &state, number t)
    {
        if (this->getInitFunction())
            return this->getInitFunction()->operator()(state, t);
        else
            return this->representation[t].getDefault();
    }

    template <class Hash, class KeyEqual>
    Pair<std::shared_ptr<State>, double> BasePointSetValueFunction<Hash, KeyEqual>::evaluate(const std::shared_ptr<State> &state, number t)
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

                // Estimate the value of the relaxation at s_k
                double v_relax_k = this->getRelaxedValueAt(s_k, t);

                // Get minimum ratio s(x)/s^k(x)
                double ratio = this->computeRatio(state, s_k);

                // Determine the "value" for k-th point
                double min_int = ratio * (v_k - v_relax_k);

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
    double BasePointSetValueFunction<Hash, KeyEqual>::computeRatio(const std::shared_ptr<State> &s, const std::shared_ptr<State> &s_k)
    {
        if (sdm::isInstanceOf<OccupancyStateInterface>(s))
        {
            return this->ratioOccupancy(sdm::to<OccupancyStateInterface>(s), sdm::to<OccupancyStateInterface>(s_k));
        }
        else if (sdm::isInstanceOf<BeliefInterface>(s))
        {
            return this->ratioBelief(sdm::to<BeliefInterface>(s), sdm::to<BeliefInterface>(s_k));
        }
        else
        {
            return 0;
        }
    }

    template <class Hash, class KeyEqual>
    void BasePointSetValueFunction<Hash, KeyEqual>::setValueAt(const std::shared_ptr<State> &state, double new_value, number t)
    {
        assert((getValueAt(state, t) < new_value) && "New value is higher than the old");
        BaseTabularValueFunction<Hash, KeyEqual>::setValueAt(state, new_value, t);
    }

    template <class Hash, class KeyEqual>
    double BasePointSetValueFunction<Hash, KeyEqual>::ratioBelief(const std::shared_ptr<BeliefInterface> &b, const std::shared_ptr<BeliefInterface> &b_k)
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
    double BasePointSetValueFunction<Hash, KeyEqual>::ratioOccupancy(const std::shared_ptr<OccupancyStateInterface> &s, const std::shared_ptr<OccupancyStateInterface> &s_k)
    {
        // Determine the ratio for the specific case when the state is a Occupancy State
        double min_ratio;
        std::shared_ptr<OccupancyStateInterface> point, occupancy_state;
        if (!this->is_sawtooth_lp)
        {
            min_ratio = 1.0;
            point = s_k;
            occupancy_state = s;
        }
        else
        {
            min_ratio = std::numeric_limits<double>::max();
            point = s_k->getOneStepUncompressedOccupancy();
            occupancy_state = s->getOneStepUncompressedOccupancy();
        }

        // Go over all joint history
        for (auto &o : point->getJointHistories())
        {
            // Go over all hidden state in the belief conditionning to the joitn history
            for (const auto &x : point->getBeliefAt(o)->getStates())
            {
                double ratio_k_o_x = (occupancy_state->getProbability(o, x) / point->getProbability(o, x));
                // determine the min int
                if (ratio_k_o_x < min_ratio)
                {
                    min_ratio = ratio_k_o_x;
                }
            }
        }
        return min_ratio;
    }

    // **********************
    // ******* Pruning ******
    // **********************

    template <class Hash, class KeyEqual>
    Pair<std::unordered_map<std::shared_ptr<State>, std::vector<std::shared_ptr<State>>>, std::map<int, std::vector<std::shared_ptr<State>>>> BasePointSetValueFunction<Hash, KeyEqual>::iterative_pruning(number t)
    {
        std::unordered_map<std::shared_ptr<State>, std::vector<std::shared_ptr<State>>> support_of_each_point;
        std::map<int, std::vector<std::shared_ptr<State>>> sort_by_number_of_support;

        auto start_representation = this->representation[t];

        // Initialise the map support_of_each_point;
        for (const auto &point_AND_value : start_representation)
        {
            support_of_each_point.emplace(point_AND_value.first, std::vector<std::shared_ptr<State>>());
        }

        // Search for the support of each point
        for (const auto &point_AND_value : start_representation)
        {
            auto evaluate = this->evaluate(point_AND_value.first, t);
            this->setValueAt(point_AND_value.first, evaluate.second, t);
            support_of_each_point[evaluate.first].push_back(point_AND_value.first);
        }

        int count = 0;
        // Sort the map "support_of_each_point" by the number of time each point is a support
        for (const auto &element : support_of_each_point)
        {
            // Delete the element that aren't useful for any other point
            if (element.second.size() == 0)
            {
                count++;
                this->representation[t].erase(element.first);
            }
            else
            {
                sort_by_number_of_support[element.second.size()].push_back(element.first);
            }
        }
        return std::make_pair(support_of_each_point, sort_by_number_of_support);
    }

    template <class Hash, class KeyEqual>
    void BasePointSetValueFunction<Hash, KeyEqual>::prune(number t)
    {
        if (this->type_of_sawtooth_prunning_ == SawtoothPrunning::BOTH or this->type_of_sawtooth_prunning_ == SawtoothPrunning::GLOBAL)
        {
            auto [support_of_each_point, sort_by_number_of_support] = this->iterative_pruning(t);

            // std::vector<std::shared_ptr<State>> to_delete;

            // Container current_representation = this->getRepresentation(t);
            // Container tempo_representation;

            // double value_without_me;
            // bool is_useful;

            // // Go over all key_value sorted by the number of time they are the support of other point
            // for(const auto&key_value : sort_by_number_of_support)
            // {
            //     // Go over each state conditionning to a precise  number of time they are support of other point
            //     for(const auto&state : key_value.second)
            //     {
            //         // If the state doesn't support any other point, we can remove it.
            //         if(key_value.first == 0)
            //         {
            //             current_representation.erase(state);
            //             continue;
            //         }

            //         // We delete temporaly the current state
            //         tempo_representation = current_representation;
            //         tempo_representation.erase(state);

            //         this->representation[t] = tempo_representation;

            //         is_useful = false;

            //         // Go over all point in which the current point is the support
            //         for(const auto &state_2 : support_of_each_point[state])
            //         {
            //             // Test the value without the current point
            //             value_without_me = this->evaluate(state_2,t).second;

            //             if(value_without_me>this->representation[t][state_2])
            //             {
            //                 is_useful = true;
            //             }

            //             if(is_useful)
            //                 break;
            //         }

            //         //If the point isn't useful, we can delete it
            //         if(!is_useful)
            //         {
            //             current_representation.erase(state);
            //             std::cout<<"Yes The global is useful"<<std::endl;
            //         }
            //     }
            // }
            // this->representation[t] = current_representation;
        }
    }

    template <class Hash, class KeyEqual>
    std::shared_ptr<ValueFunctionInterface> BasePointSetValueFunction<Hash, KeyEqual>::copy()
    {
        auto casted_value = std::dynamic_pointer_cast<BasePointSetValueFunction<Hash, KeyEqual>>(this->getptr());
        return std::make_shared<BasePointSetValueFunction<Hash, KeyEqual>>(*casted_value);
    }

    template <class Hash, class KeyEqual>
    std::string BasePointSetValueFunction<Hash, KeyEqual>::str() const
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
                state_str << pair_st_val.first->str();
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
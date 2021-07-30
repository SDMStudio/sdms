#include <sdm/utils/value_function/point_set_value_function.hpp>
#include <sdm/utils/value_function/backup/backup_base.hpp>
#include <sdm/core/state/interface/belief_interface.hpp>
#include <sdm/core/state/interface/occupancy_state_interface.hpp>

namespace sdm
{
    template <class Hash, class KeyEqual>
    BasePointSetValueFunction<Hash, KeyEqual>::BasePointSetValueFunction(number horizon, const std::shared_ptr<Initializer> &initializer, const std::shared_ptr<BackupInterfaceForValueFunction> &backup, const std::shared_ptr<ActionVFInterface> &action_vf, int freq_pruning, TypeOfSawtoothPrunning type_of_sawtooth_prunning)
        : BaseTabularValueFunction<Hash, KeyEqual>(horizon, initializer, backup, action_vf), freq_pruning_(freq_pruning), type_of_sawtooth_prunning_(type_of_sawtooth_prunning)
    {
    }

    template <class Hash, class KeyEqual>
    BasePointSetValueFunction<Hash, KeyEqual>::BasePointSetValueFunction(number horizon, double default_value, const std::shared_ptr<BackupInterfaceForValueFunction> &backup, const std::shared_ptr<ActionVFInterface> &action_vf, int freq_pruning, TypeOfSawtoothPrunning type_of_sawtooth_prunning)
        : BasePointSetValueFunction<Hash, KeyEqual>(horizon, std::make_shared<ValueInitializer>(default_value), backup, action_vf, freq_pruning, type_of_sawtooth_prunning)
    {
    }

    template <class Hash, class KeyEqual>
    double BasePointSetValueFunction<Hash, KeyEqual>::getValueAt(const std::shared_ptr<State> &state, number t)
    {
        return (t >= this->getHorizon()) ? this->representation[t].getDefault() : this->evaluate(state, t).second;
    }

    template <class Hash, class KeyEqual>
    void BasePointSetValueFunction<Hash, KeyEqual>::updateValueAt(const std::shared_ptr<State> &state, number t)
    {
        BaseTabularValueFunction<Hash,KeyEqual>::updateValueAt(state,t);

        if(this->type_of_sawtooth_prunning_ == TypeOfSawtoothPrunning::BOTH or this->type_of_sawtooth_prunning_ == TypeOfSawtoothPrunning::ITERATIVE)
        {
            this->iterative_pruning(t);
        }
    }

    template <class Hash, class KeyEqual>
    std::string BasePointSetValueFunction<Hash, KeyEqual>::str() const
    {
        std::ostringstream res;
        res << "<point_set_representation, horizon=\"" << ((this->isInfiniteHorizon()) ? "inf" : std::to_string(this->getHorizon())) << "\">" << std::endl;
        for (std::size_t i = 0; i < this->representation.size(); i++)
        {
            res << "\t<value timestep=\"" << ((this->isInfiniteHorizon()) ? "all" : std::to_string(i)) << "\" default=\"" << this->representation[i].getDefault() << "\">" << std::endl;
            for (const auto &pair_st_val : this->representation[i])
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

    // template <class Hash, class KeyEqual>
    // Pair<std::shared_ptr<State>, double> BasePointSetValueFunction<Hash, KeyEqual>::evaluate(const std::shared_ptr<State> &state_tmp, number t)
    // {
    //     assert(this->getInitFunction() != nullptr);
    //     assert(state_tmp->getTypeState() != TypeState::STATE);

    //     auto state = state_tmp->toBelief();

    //     double min_ext = 0;
    //     double v_ub_state = this->getInitFunction()->operator()(state, t);

    //     std::shared_ptr<State> argmin_ = state;

    //     // Go over all element in the support
    //     for (const auto &point_value : this->getRepresentation(t))
    //     {
    //         auto [point, v_kappa] = point_value;
    //         auto point_to_belief_interface = point->toBelief();

    //         double v_ub_kappa = this->getInitFunction()->operator()(point, t);

    //         double phi;

    //         switch (state->getTypeState())
    //         {
    //         case TypeState::BELIEF_STATE:
    //             phi = this->ratioBelief(state, point_to_belief_interface);
    //             break;
    //         case TypeState::OCCUPANCY_STATE:
    //             phi = this->ratioOccupancy(state, point_to_belief_interface,t);
    //             break;
    //         case TypeState::SERIAL_OCCUPANCY_STATE:
    //             phi = this->ratioOccupancy(state, point_to_belief_interface,t);
    //             break;
    //         default:
    //             throw sdm::exception::Exception("BasePointSetValueFunction::evaluate not defined for this state!");
    //             break;
    //         }

    //         // determine the min ext
    //         double min_int = phi * (v_kappa - v_ub_kappa);
    //         if (min_int < min_ext)
    //         {
    //             min_ext = min_int;
    //             argmin_ = point_to_belief_interface;
    //         }
    //     }
    //     return std::make_pair(argmin_, v_ub_state + min_ext);
    // }

    // template <class Hash, class KeyEqual>
    // double BasePointSetValueFunction<Hash, KeyEqual>::ratioBelief(const std::shared_ptr<BeliefInterface> &state, const std::shared_ptr<BeliefInterface> &point)
    // {
    //     // Determine the ratio for the specific case when the state is a belief
    //     double phi = 1.0;

    //     for (auto &support : point->getStates())
    //     {
    //         double v_int = (state->getProbability(support) / point->getProbability(support));
    //         // determine the min int
    //         if (v_int < phi)
    //         {
    //             phi = v_int;
    //         }
    //     }
    //     return phi;
    // }

    // template <class Hash, class KeyEqual>
    // double BasePointSetValueFunction<Hash, KeyEqual>::ratioOccupancy(const std::shared_ptr<BeliefInterface> &state_tmp, const std::shared_ptr<BeliefInterface> &point_tmp, number t)
    // {
    //     // Determine the ratio for the specific case when the state is a Occupancy State
    //     double phi = 1.0;
    //     auto point = point_tmp->toOccupancyState();
    //     auto occupancy_state = state_tmp->toOccupancyState();

    //     // Go over all joint history
    //     for (auto &joint_history : point->getJointHistories())
    //     {
    //         // Go over all hidden state in the belief conditionning to the joitn history
    //         for (const auto &hidden_state : point->getBeliefAt(joint_history)->getStates())
    //         {
    //             double v_int = (occupancy_state->getProbability(joint_history, hidden_state) / point->getProbability(joint_history, hidden_state));
    //             // determine the min int
    //             if (v_int < phi)
    //             {
    //                 phi = v_int;
    //             }
    //         }
    //     }
    //     return phi;
    // }

    template <class Hash, class KeyEqual>
    Pair<std::shared_ptr<State>, double> BasePointSetValueFunction<Hash, KeyEqual>::evaluate(const std::shared_ptr<State> &state_tmp, number t)
    {
        assert(this->getInitFunction() != nullptr);
        assert(state_tmp->getTypeState() != TypeState::STATE);

        auto state = state_tmp->toBelief();

        // double min_ext = 0;
        double min_value = 0;
        
        if(this->getSupport(t).size() != 0)
        {
            min_value = std::numeric_limits<double>::max();
        }

        double v_ub_state = this->getInitFunction()->operator()(state, t);

        std::shared_ptr<State> argmin_ = state;

        // Go over all element in the support
        for (const auto &point_value : this->getRepresentation(t))
        {
            auto [point, v_kappa] = point_value;
            auto point_to_belief_interface = point->toBelief();

            double v_ub_kappa = this->getInitFunction()->operator()(point->toOccupancyState()->getOneStepUncompressedOccupancy(), t);

            double phi;

            switch (state->getTypeState())
            {
            case TypeState::BELIEF_STATE:
                phi = this->ratioBelief(state, point_to_belief_interface);
                break;
            case TypeState::OCCUPANCY_STATE:
                phi = this->ratioOccupancy(state, point_to_belief_interface,t);
                break;
            case TypeState::SERIAL_OCCUPANCY_STATE:
                phi = this->ratioOccupancy(state, point_to_belief_interface,t);
                break;
            default:
                throw sdm::exception::Exception("BasePointSetValueFunction::evaluate not defined for this state!");
                break;
            }

            // determine the min ext
            double min_int = phi * (v_kappa - v_ub_kappa);
            if (min_int < min_value)
            {
                min_value = min_int;
                argmin_ = point_to_belief_interface;
            }
        }
        return std::make_pair(argmin_,v_ub_state+ min_value);
    }

    template <class Hash, class KeyEqual>
    double BasePointSetValueFunction<Hash, KeyEqual>::ratioBelief(const std::shared_ptr<BeliefInterface> &state, const std::shared_ptr<BeliefInterface> &point)
    {
        // Determine the ratio for the specific case when the state is a belief
        double phi = 1.0;

        for (auto &support : point->getStates())
        {
            double v_int = (state->getProbability(support) / point->getProbability(support));
            // determine the min int
            if (v_int < phi)
            {
                phi = v_int;
            }
        }
        return phi;
    }

    template <class Hash, class KeyEqual>
    double BasePointSetValueFunction<Hash, KeyEqual>::ratioOccupancy(const std::shared_ptr<BeliefInterface> &state_tmp, const std::shared_ptr<BeliefInterface> &point_tmp, number t)
    {
        // Determine the ratio for the specific case when the state is a Occupancy State
        // double phi = 1.0;
        double min_value = std::numeric_limits<double>::max();

        auto point = point_tmp->toOccupancyState()->getOneStepUncompressedOccupancy();
        auto occupancy_state = state_tmp->toOccupancyState()->getOneStepUncompressedOccupancy();

        // Go over all joint history
        for (auto &joint_history : point->getJointHistories())
        {
            // Go over all hidden state in the belief conditionning to the joitn history
            for (const auto &hidden_state : point->getBeliefAt(joint_history)->getStates())
            {
                double v_int = (occupancy_state->getProbability(joint_history, hidden_state) / point->getProbability(joint_history, hidden_state));
                // determine the min int
                if (min_value > v_int)
                {
                    min_value = v_int;
                }
            }
        }
        return min_value;
    }

    // **********************
    // ****** Prunning ******
    // **********************

    template <class Hash, class KeyEqual>
    void BasePointSetValueFunction<Hash, KeyEqual>::do_pruning(number t)
    {
        if (t%this->freq_pruning_ == 0)
        {
            for (number time = 0; time < this->getHorizon(); time++)
            {
                this->prune(time);
            }
        }
    }

    template <class Hash, class KeyEqual>
    Pair<std::unordered_map<std::shared_ptr<State>,std::vector<std::shared_ptr<State>>>,std::map<int,std::vector<std::shared_ptr<State>>>> BasePointSetValueFunction<Hash, KeyEqual>::iterative_pruning(number t)
    {
        std::unordered_map<std::shared_ptr<State>,std::vector<std::shared_ptr<State>>> support_of_each_point; 
        std::map<int,std::vector<std::shared_ptr<State>>> sort_by_number_of_support; 

        Container start_representation = this->getRepresentation(t);

        // Initialise the map support_of_each_point;
        for(const auto &point_AND_value : start_representation)
        {
            support_of_each_point.emplace(point_AND_value.first,std::vector<std::shared_ptr<State>>());
        }

        // Search for the support of each point
        for(const auto &point_AND_value : start_representation)
        {
            auto evaluate = this->evaluate(point_AND_value.first,t);
            BaseTabularValueFunction<Hash,KeyEqual>::updateValueAt(point_AND_value.first,t,evaluate.second);
            support_of_each_point[evaluate.first].push_back(point_AND_value.first);
        }

        // Sort the map "support_of_each_point" by the number of time each point is a support
        for(const auto &element : support_of_each_point)
        {
            // Delete the element that aren't useful for any other point
            if(element.second.size() == 0 )
            {
                this->representation[t].erase(element.first);
            }else
            {
                sort_by_number_of_support[element.second.size()].push_back(element.first);
            }
        }
        return std::make_pair(support_of_each_point,sort_by_number_of_support);
    }

    template <class Hash, class KeyEqual>
    void BasePointSetValueFunction<Hash, KeyEqual>::prune(number t)
    {
        if(this->type_of_sawtooth_prunning_ == TypeOfSawtoothPrunning::BOTH or this->type_of_sawtooth_prunning_ == TypeOfSawtoothPrunning::GLOBAL)
        {
            auto [support_of_each_point,sort_by_number_of_support] = this->iterative_pruning(t);

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
} // namespace sdm
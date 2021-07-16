#include <sdm/utils/value_function/point_set_value_function.hpp>
#include <sdm/utils/value_function/backup/backup_base.hpp>
#include <sdm/core/state/interface/belief_interface.hpp>
#include <sdm/core/state/interface/occupancy_state_interface.hpp>

namespace sdm
{
    PointSetValueFunction::PointSetValueFunction(number horizon, const std::shared_ptr<Initializer> &initializer,const std::shared_ptr<BackupInterfaceForValueFunction> &backup, const std::shared_ptr<ActionVFInterface> &action_vf,int freq_prunning )
        : TabularValueFunction(horizon, initializer, backup,action_vf), freq_prune_(freq_prunning)
    {
    }

    PointSetValueFunction::PointSetValueFunction(number horizon, double default_value, const std::shared_ptr<BackupInterfaceForValueFunction> &backup, const std::shared_ptr<ActionVFInterface> &action_vf,int freq_prunning)
        : TabularValueFunction(horizon, std::make_shared<ValueInitializer>(default_value), backup, action_vf),freq_prune_(freq_prunning)
    {
    }

    double PointSetValueFunction::getValueAt(const std::shared_ptr<State> &state, number t)
    {
        assert(this->getInitFunction() != nullptr);

        // Determine if the element exist 
        bool already_exist = false;
        for (auto iter = this->representation[t].begin(); iter != this->representation[t].end(); iter++)
        {
            if (iter->first == state)
            {
                already_exist = true;
            }
        }
        // If the element doesn't exit, we determine this value else with take the value stocked
        return (already_exist or (t>=this->getHorizon()) ) ? this->representation[t].at(state) : this->evaluate(state, t).second;
    }

    void PointSetValueFunction::updateValueAt(const std::shared_ptr<State> &state, number t, double target)
    {
        TabularValueFunction::updateValueAt(state, t, target);
    }

    std::string PointSetValueFunction::str() const
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

    Pair<std::shared_ptr<State>,double> PointSetValueFunction::evaluate(const std::shared_ptr<State>& state_tmp, number t)
    {
        assert(this->getInitFunction() != nullptr);
        assert(state_tmp->getTypeState() != TypeState::STATE);

        auto state = state_tmp->toBelief();

        double min_ext = 0;
        double v_ub_state = this->getInitFunction()->operator()(state, t);

        std::shared_ptr<State> argmin_ = state;

        // Go over all element in the support
        for (const auto &point_value : this->getRepresentation(t))
        {
            auto [point,v_kappa] = point_value;
            auto point_to_belief_interface = point->toBelief();

            double v_ub_kappa = this->getInitFunction()->operator()(point, t);
            
            double phi;

            switch (state->getTypeState())
            {
            case TypeState::BELIEF_STATE :
                phi = this->ratioBelief(state,point_to_belief_interface);
                break;
            case TypeState::OCCUPANCY_STATE :
                phi = this->ratioOccupancy(state,point_to_belief_interface);
                break;
            case TypeState::SERIAL_OCCUPANCY_STATE :
                phi = this->ratioOccupancy(state,point_to_belief_interface);
                break;
            default:
                throw sdm::exception::Exception("PointSetValueFunction::evaluate not defined for this state!");
                break;
            }

            // determine the min ext
            double min_int = phi * (v_kappa - v_ub_kappa);
            if (min_int < min_ext)
            {
                min_ext = min_int;
                argmin_ = point_to_belief_interface;
            }
        }
        return std::make_pair(argmin_,v_ub_state + min_ext);
    }

    double PointSetValueFunction::ratioBelief(const std::shared_ptr<BeliefInterface>& state, const std::shared_ptr<BeliefInterface>& point)
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

    double PointSetValueFunction::ratioOccupancy(const std::shared_ptr<BeliefInterface>& state_tmp, const std::shared_ptr<BeliefInterface>& point_tmp)
    {
        // Determine the ratio for the specific case when the state is a Occupancy State
        
        double phi = 1.0;

        auto point = point_tmp->toOccupancyState();
        auto occupancy_state = state_tmp->toOccupancyState();

        // Go over all joint history
        for (auto &joint_history : point->getJointHistories())
        {
            // Go over all hidden state in the belief conditionning to the joitn history
            for(const auto& hidden_state :  point->getBeliefAt(joint_history)->getStates())
            {
                double v_int = (occupancy_state->getProbability(joint_history,hidden_state) / point->getProbability(joint_history,hidden_state));
                // determine the min int
                if (v_int < phi)
                {
                    phi = v_int;
                }
            }
        }
        return phi;
    }

    // **********************
    // ****** Prunning ******
    // **********************

    void PointSetValueFunction::do_prunning(number t)
    {
        if (this->last_prunning == this->freq_prune_)
        {
            for (number time = 0; time < this->getHorizon(); time++)
            {
                this->prune(time);
            }
            this->last_prunning = 0;
        }
        this->last_prunning++;
    }

    void PointSetValueFunction::prune(number t)
    {
        // Pour le moment, la méthode n'est pas efficace 
        // On peut améliorer cela, en effecutant une première boucle qui note le nombre de fois où le point est le support de quelqu'un
        // Et grâce à cela, on refait la suite de mon code, mais à la place, on parcourt les points qui ont pour support notre points 
        // (dans l'ordre décroissant )


        std::vector<std::shared_ptr<State>> to_delete;

        Container start_representation = this->getRepresentation(t);
        Container current_representation = start_representation;
        Container tempo_representation;

        double value_without_me;
        bool is_useful;

        // Go over all point 
        for(const auto &point_AND_value : start_representation)
        {
            // Delete the current point in order to test if it's useful to another point 
            tempo_representation = current_representation;
            tempo_representation.erase(point_AND_value.first);

            this->representation[t] = tempo_representation;

            is_useful = false;

            //Go over all point 
            for(const auto &point_AND_value_2 : current_representation)
            {
                // Test the value without the current point
                value_without_me = this->evaluate(point_AND_value_2.first,t).second;

                if(value_without_me>point_AND_value_2.second)
                {
                    is_useful = true;
                }

                if(is_useful)
                    break;
            }

            if(!is_useful)
                current_representation.erase(point_AND_value.first);
        }

        this->representation[t] = current_representation;
    }

    // bool PointSetValueFunction::is_dominated(const std::shared_ptr<State> &state, double value, number t)
    // {
    //     auto pair_witness_ostate = this->evaluate(state,t);

    //     if (pair_witness_ostate.first == state)
    //     {
    //         return false;
    //     }
    //     else
    //     {
    //         return (pair_witness_ostate.second > value);// + this->epsilon_prunning);
    //     }
    // }
} // namespace sdm
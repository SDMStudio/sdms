#include <sdm/utils/value_function/point_set_value_function.hpp>
#include <sdm/utils/value_function/backup/backup_base.hpp>
#include <sdm/core/state/interface/belief_interface.hpp>
#include <sdm/core/state/interface/occupancy_state_interface.hpp>

namespace sdm
{
    template <class Hash, class KeyEqual>
    BasePointSetValueFunction<Hash, KeyEqual>::BasePointSetValueFunction(number horizon, const std::shared_ptr<Initializer> &initializer, const std::shared_ptr<BackupInterfaceForValueFunction> &backup, const std::shared_ptr<ActionVFInterface> &action_vf, int freq_prunning)
        : BaseTabularValueFunction<Hash, KeyEqual>(horizon, initializer, backup, action_vf), freq_prune_(freq_prunning)
    {
    }

    template <class Hash, class KeyEqual>
    BasePointSetValueFunction<Hash, KeyEqual>::BasePointSetValueFunction(number horizon, double default_value, const std::shared_ptr<BackupInterfaceForValueFunction> &backup, const std::shared_ptr<ActionVFInterface> &action_vf, int freq_prunning)
        : BaseTabularValueFunction<Hash, KeyEqual>(horizon, std::make_shared<ValueInitializer>(default_value), backup, action_vf), freq_prune_(freq_prunning)
    {
    }

    template <class Hash, class KeyEqual>
    double BasePointSetValueFunction<Hash, KeyEqual>::getValueAt(const std::shared_ptr<State> &state, number t)
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
        return (already_exist or (t >= this->getHorizon())) ? this->representation[t].at(state) : this->evaluate(state, t).second;
    }

    template <class Hash, class KeyEqual>
    void BasePointSetValueFunction<Hash, KeyEqual>::updateValueAt(const std::shared_ptr<State> &state, number t, double target)
    {
        BaseTabularValueFunction<Hash, KeyEqual>::updateValueAt(state, t, target);

        if (this->last_prunning == this->freq_prune_)
        {
            for (number time = 0; time < this->getHorizon(); time++)
            {
                // this->prune(time);
            }
            this->last_prunning = 0;
        }
        this->last_prunning++;
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

    template <class Hash, class KeyEqual>
    void BasePointSetValueFunction<Hash, KeyEqual>::prune(number t)
    {
        std::vector<std::shared_ptr<State>> to_delete;

        for (auto iter = this->representation[t].begin(); iter != this->representation[t].end(); iter++)
        {
            if (this->is_dominated(iter->first, iter->second, t))
            {
                to_delete.push_back(iter->first);
            }
        }

        for (const auto &i : to_delete)
        {
            this->representation[t].erase(i);
        }
    }

    template <class Hash, class KeyEqual>
    bool BasePointSetValueFunction<Hash, KeyEqual>::is_dominated(const std::shared_ptr<State> &state, double value, number t)
    {
        auto pair_witness_ostate = this->evaluate(state, t);

        if (pair_witness_ostate.first == state)
        {
            return false;
        }
        else
        {
            return (pair_witness_ostate.second <= value + this->epsilon_prunning);
        }
    }

    template <class Hash, class KeyEqual>
    Pair<std::shared_ptr<State>, double> BasePointSetValueFunction<Hash, KeyEqual>::evaluate(const std::shared_ptr<State> &state_tmp, number t)
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
            auto [point, v_kappa] = point_value;
            auto point_to_belief_interface = point->toBelief();

            double v_ub_kappa = this->getInitFunction()->operator()(point, t);

            double phi;

            switch (state->getTypeState())
            {
            case TypeState::BELIEF_STATE:
                phi = this->ratioBelief(state, point_to_belief_interface);
                break;
            case TypeState::OCCUPANCY_STATE:
                phi = this->ratioOccupancy(state, point_to_belief_interface);
                break;
            case TypeState::SERIAL_OCCUPANCY_STATE:
                phi = this->ratioOccupancy(state, point_to_belief_interface);
                break;
            default:
                throw sdm::exception::Exception("BasePointSetValueFunction::evaluate not defined for this state!");
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
        return std::make_pair(argmin_, v_ub_state + min_ext);
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
    double BasePointSetValueFunction<Hash, KeyEqual>::ratioOccupancy(const std::shared_ptr<BeliefInterface> &state_tmp, const std::shared_ptr<BeliefInterface> &point_tmp)
    {
        // Determine the ratio for the specific case when the state is a Occupancy State

        double phi = 1.0;

        auto point = point_tmp->toOccupancyState();
        auto occupancy_state = state_tmp->toOccupancyState();

        // Go over all joint history
        for (auto &joint_history : point->getJointHistories())
        {
            // Go over all hidden state in the belief conditionning to the joitn history
            for (const auto &hidden_state : point->getBeliefAt(joint_history)->getStates())
            {
                double v_int = (occupancy_state->getProbability(joint_history, hidden_state) / point->getProbability(joint_history, hidden_state));
                // determine the min int
                if (v_int < phi)
                {
                    phi = v_int;
                }
            }
        }
        return phi;
    }

} // namespace sdm
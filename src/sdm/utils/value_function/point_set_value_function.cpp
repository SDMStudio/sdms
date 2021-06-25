#include <sdm/utils/value_function/point_set_value_function.hpp>
#include <sdm/utils/value_function/backup/backup_base.hpp>

namespace sdm
{
    PointSetValueFunction::PointSetValueFunction(number horizon, const std::shared_ptr<Initializer> &initializer,const std::shared_ptr<BackupInterface<double>> &backup, const std::shared_ptr<ActionVFInterface<double>> &action_vf, const std::shared_ptr<EvaluateVFInterface> &evaluate,int freq_prunning )
        : TabularValueFunction(horizon, initializer, backup,action_vf,evaluate), freq_prune_(freq_prunning)
    {
    }

    PointSetValueFunction::PointSetValueFunction(number horizon, double default_value, const std::shared_ptr<BackupInterface<double>> &backup, const std::shared_ptr<ActionVFInterface<double>> &action_vf, const std::shared_ptr<EvaluateVFInterface> &evaluate,int freq_prunning)
        : TabularValueFunction(horizon, std::make_shared<ValueInitializer>(default_value), backup, action_vf,evaluate),freq_prune_(freq_prunning)
    {
    }

    void PointSetValueFunction::updateValueAt(const std::shared_ptr<State> &state, number t, double target)
    {
        TabularValueFunction::updateValueAt(state, t, target);

        // 
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
                state_str << pair_st_val.first;
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

    void PointSetValueFunction::prune(number t)
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

    bool PointSetValueFunction::is_dominated(const std::shared_ptr<State> &state, double value, number t)
    {
        auto pair_witness_ostate = this->evaluate(state,t);

        if (pair_witness_ostate.first == state)
        {
            return false;
        }
        else
        {
            return (pair_witness_ostate.second <= value + this->epsilon_prunning);
        }
    }

} // namespace sdm
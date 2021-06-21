#include <sdm/utils/value_function/hyperplan_value_function.hpp>
#include <sdm/utils/value_function/backup/backup_base.hpp>
#include <sdm/core/state/interface/belief_interface.hpp>
#include <sdm/core/state/belief_default.hpp>

namespace sdm
{        
    HyperplanValueFunction::HyperplanValueFunction(number horizon, const std::shared_ptr<Initializer> &initializer,const std::shared_ptr<BackupInterface> &backup, int freq_prunning)
        : ValueFunction(horizon, initializer, backup), freq_prune_(freq_prunning)
    {
        this->representation = std::vector<HyperplanSet>(this->isInfiniteHorizon() ? 1 : this->horizon_ + 1, HyperplanSet({}));
        this->default_values_per_horizon = std::vector<double>(this->isInfiniteHorizon() ? 1 : this->horizon_ + 1, 0);
    }

    HyperplanValueFunction::HyperplanValueFunction(number horizon, double default_value,const std::shared_ptr<BackupInterface> &backup, int freq_prunning)
        : HyperplanValueFunction(horizon, std::make_shared<ValueInitializer>(default_value),backup,freq_prunning){}

    HyperplanValueFunction::~HyperplanValueFunction(){}

    void HyperplanValueFunction::initialize(double value, number t)
    {
        auto new_v = std::make_shared<BeliefDefault>(value);
        this->representation[t].push_back(new_v);
        this->default_values_per_horizon[t] = value;
    }

    void HyperplanValueFunction::initialize()
    {
        this->initializer_->init(this->getptr());
    }

    double HyperplanValueFunction::getValueAt(const std::shared_ptr<State> &state, number t)
    {
        return std::static_pointer_cast<BackupBase<std::shared_ptr<State>>>(this->backup_)->getMaxAt(this->getptr(), state, t).first;
    }

    void HyperplanValueFunction::updateValueAt(const std::shared_ptr<State> &state, number t)
    {
        const auto &new_hyperplan = std::static_pointer_cast<BackupBase<std::shared_ptr<State>>>(this->backup_)->backup(this->getptr(),state,t);

        if (std::find(this->representation[t].begin(), this->representation[t].end(), new_hyperplan) == this->representation[t].end())
            this->representation[t].push_back(new_hyperplan);

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

    
    std::vector<std::shared_ptr<State>> HyperplanValueFunction::getSupport(number t)
    {
        return this->representation[t];
    }

    double HyperplanValueFunction::getDefaultValue(number t)
    {
        return this->default_values_per_horizon[t];
    }
    
    void HyperplanValueFunction::prune(number )
    {
        // this->pairwise_prune(t);
    }

    void HyperplanValueFunction::pairwise_prune(number t)
    {
        std::vector<std::shared_ptr<BeliefInterface>> hyperplan_not_to_be_deleted;
        std::vector<std::shared_ptr<BeliefInterface>> hyperplan_to_delete;

        // Go over all hyperplan
        for (const auto &alpha : this->getSupport(t))
        {
            bool alpha_dominated = false;

            //Go over all hyperplan in hyperplan_not_to_be_deleted
            for (const auto &beta : hyperplan_not_to_be_deleted)
            {
                // If beta dominate alpha, we had alpha to the hyperplan to delete
                if (alpha<beta)
                {
                    hyperplan_to_delete.push_back(alpha->toBelief());
                    alpha_dominated = true;
                    break;
                }
            }
            // If alpha is dominated, we go to the next hyperplan
            if (alpha_dominated)
            {
                continue;
            }

            //Go over all hyperplan in hyperplan_not_to_be_deleted
            std::vector<std::shared_ptr<BeliefInterface>> erase_tempo;

            for(const auto &beta : hyperplan_not_to_be_deleted)
            {
                //If alpha dominate a vector in hyperplan_not_to_be_deleted, we deleted this vector
                if (beta<alpha)
                {
                    erase_tempo.push_back(beta);
                }
            }

            for (const auto &erase : erase_tempo)
            {
                auto it = std::find(hyperplan_not_to_be_deleted.begin(), hyperplan_not_to_be_deleted.end(), erase);
                hyperplan_not_to_be_deleted.erase(it);
            }
            hyperplan_not_to_be_deleted.push_back(alpha->toBelief());
        }

        for(const auto &to_delete : hyperplan_to_delete)
        {
            this->representation[t].erase(std::find(this->representation[t].begin(), this->representation[t].end(), to_delete));
        }
    }
    
    void HyperplanValueFunction::bounded_prune(number )
    {
        // std::unordered_map<std::shared_ptr<State>, number> refCount;
        // auto all_plan = this->isInfiniteHorizon() ? this->representation[0] : this->representation[t];

        // // Initialize ref count to 0 for each hyperplan
        // for (auto iter = all_plan.begin(); iter != all_plan.end(); iter++)
        // {
        //     refCount.emplace(*iter, 0);
        // }

        // //<! update the count
        // std::shared_ptr<State> max_alpha;
        // double max_value = -std::numeric_limits<double>::max(), value;
        // for (const auto &hyperplan : all_plan)
        // {
        //     for (const auto &alpha : refCount)
        //     {
        //         if (max_value < (value = (hyperplan) ^ (alpha.first)))
        //         {
        //             max_value = value;
        //             max_alpha = alpha.first;
        //         }
        //     }

        //     if(refCount.find(max_alpha) != refCount.end())
        //     {
        //         refCount.at(max_alpha)++;
        //     }
        // }

        // for (auto iter = all_plan.begin(); iter != all_plan.end(); iter++)
        // {
        //     if (refCount.at(*iter) == 0)
        //     {
        //         this->representation[t].erase(std::find(this->representation[t].begin(), this->representation[t].end(), *iter));
        //     }
        // }
    }

    std::string HyperplanValueFunction::str() const
    {
        std::ostringstream res;
        res << "<hyperplan_value_function horizon=\"" << ((this->isInfiniteHorizon()) ? "inf" : std::to_string(this->getHorizon())) << "\">" << std::endl;

        for (number i = 0; i < this->representation.size(); i++)
        {
            res << "\t<value timestep=\"" << ((this->isInfiniteHorizon()) ? "all" : std::to_string(i)) << ">" << std::endl;
            for (auto plan : this->representation[i])
            {
                res << "\t\t<plan>" << std::endl;
                res << "\t\t\t" << plan->str() << std::endl;
                res << "\t\t</plan>" << std::endl;
            }
            res << "\t</value>" << std::endl;
        }

        res << "</hyerplan_value_function>" << std::endl;
        return res.str();
    }
} // namespace sdm
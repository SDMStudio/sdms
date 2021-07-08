#include <sdm/utils/value_function/hyperplan_value_function.hpp>
#include <sdm/utils/value_function/backup/backup_base.hpp>
#include <sdm/core/state/interface/belief_interface.hpp>

#include <sdm/core/state/belief_state.hpp>
#include <sdm/core/state/belief_default.hpp>

#include <sdm/core/state/occupancy_state.hpp>

namespace sdm
{        

    double HyperplanValueFunction::PRECISION = config::PRECISION_SDMS_VECTOR;

    HyperplanValueFunction::HyperplanValueFunction(number horizon, const std::shared_ptr<Initializer> &initializer, const std::shared_ptr<BackupInterfaceForValueFunction> &backup, const std::shared_ptr<ActionVFInterface> &action_vf, int freq_prunning)
        : ValueFunction(horizon, initializer,backup,action_vf), freq_prune_(freq_prunning)
    {
        this->representation = std::vector<HyperplanSet>(this->isInfiniteHorizon() ? 1 : this->horizon_ + 1, HyperplanSet({}));
        this->default_values_per_horizon = std::vector<double>(this->isInfiniteHorizon() ? 1 : this->horizon_ + 1, 0);
    }

    HyperplanValueFunction::HyperplanValueFunction(number horizon,double default_value, const std::shared_ptr<BackupInterfaceForValueFunction> &backup, const std::shared_ptr<ActionVFInterface> &action_vf, int freq_prunning)
        : HyperplanValueFunction(horizon, std::make_shared<ValueInitializer>(default_value),backup,action_vf,freq_prunning){}

    HyperplanValueFunction::~HyperplanValueFunction(){}

    void HyperplanValueFunction::initialize(double value, number t)
    {
        this->default_values_per_horizon[t] = value;
    }
    
    void HyperplanValueFunction::initialize()
    {
        this->initializer_->init(this->getptr());
    }

    double HyperplanValueFunction::getValueAt(const std::shared_ptr<State> &state, number t)
    {
        return this->evaluate(state, t).second;
    }

    void HyperplanValueFunction::updateValueAt(const std::shared_ptr<State> &state, number t)
    {
        const auto &new_hyperplan = this->template backup<std::shared_ptr<State>>(state,this->getBestAction(state,t),t)->toBelief();

        if (!this->exist(new_hyperplan->getVectorInferface(),t))
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

    bool HyperplanValueFunction::exist(const std::shared_ptr<VectorInterface<std::shared_ptr<State>,double>>& new_vector,number t, double precision)
    {
        for(const auto& element : this->representation[t])
        {
            auto vecto = element->toBelief()->getVectorInferface();

            if(vecto->size() != new_vector->size())
            {
                continue;
            }

            bool same_as_vecto = true;
            for (const auto &index : vecto->getIndexes())
            {
                if(new_vector->getValueAt(index) != vecto->getValueAt(index))
                {
                    same_as_vecto = false;
                    break;
                }
            }

            if(same_as_vecto== true)
            {
                return true;
            }
        }
        return false;
    }

    Pair<std::shared_ptr<State>,double> HyperplanValueFunction::evaluate(const std::shared_ptr<State> &state, number t)
    {
        double current, max = -std::numeric_limits<double>::max();
        std::shared_ptr<BeliefInterface> alpha_vector;

        auto belief_state = state->toBelief();

        this->createDefault(state,t);

        for (const auto &plan : this->getSupport(t))
        {
            auto belief_plan = plan->toBelief();

            if (max < (current = belief_state->operator^(belief_plan) ))
            {
                max = current;
                alpha_vector = belief_plan;
            }
        }
        return {alpha_vector,max};
    }

    void HyperplanValueFunction::createDefault(const std::shared_ptr<State>& state, number t)
    {
        if(this->representation[t].size() == 0)
        {
            std::shared_ptr<BeliefInterface> default_state;

            switch (state->getTypeState())
            {
            case TypeState::BELIEF_STATE:
                default_state =  std::make_shared<Belief>();
                break;
            case TypeState::OCCUPANCY_STATE:
                default_state =  std::make_shared<OccupancyState>();
                break;
            default:
                break;
            }

            default_state->setDefaultValue(this->getDefaultValue(t));
            this->representation[t].push_back(default_state);
        }
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

    size_t HyperplanValueFunction::getSize(number t) const
    {
        return this->representation[this->isInfiniteHorizon() ? 0 : t].size();
    }
} // namespace sdm
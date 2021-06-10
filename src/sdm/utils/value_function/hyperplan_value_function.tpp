
namespace sdm
{    
    HyperplanValueFunction::HyperplanValueFunction() {}
    
    HyperplanValueFunction::HyperplanValueFunction(number horizon, std::shared_ptr<Initializer> initializer, int freq_prunning)
        : ValueFunctionNewInterface(horizon), initializer_(initializer), freq_prune_(freq_prunning)
    {
        this->representation = std::vector<HyperplanSet>(this->isInfiniteHorizon() ? 1 : this->horizon_ + 1, HyperplanSet({}));
        this->default_values_per_horizon = std::vector<double>(this->isInfiniteHorizon() ? 1 : this->horizon_ + 1, 0);
    }

    HyperplanValueFunction::HyperplanValueFunction(number horizon, double default_value, int freq_prunning) : HyperplanValueFunction(horizon, std::make_shared<ValueInitializer>(default_value),freq_prunning){}
    HyperplanValueFunction::~HyperplanValueFunction(){}


    void HyperplanValueFunction::initialize(double value, number t)
    {
        std::shared_ptr<State> new_v(value);
        this->representation[t].push_back(new_v);
        this->default_values_per_horizon[t] = value;
    }

    void HyperplanValueFunction::initialize()
    {
        this->initializer_->init(this->getptr());
    }

    std::pair<double, std::shared_ptr<State>> HyperplanValueFunction::getMaxAt(const std::shared_ptr<State> &state, number t)
    {
        double current, max = -std::numeric_limits<double>::max();
        std::shared_ptr<State> alpha_vector;

        for (const auto &plan : this->representation[t])
        {
            current = state ^ plan;

            if (max < current)
            {
                max = current;
                alpha_vector = plan;
            }
        }

        return {max, alpha_vector};
    }

    double HyperplanValueFunction::getValueAt(const std::shared_ptr<State> &state, number t)
    {
        return this->getMaxAt(state, t).first;
    }

    void HyperplanValueFunction::updateValueAt(const std::shared_ptr<State> &state, number t)
    {
        const auto &new_hyperplan = std::static_pointer_cast<BackupBase<std::shared_ptr<State>>>(this->backup_)->backup(std::make_shared<ValueFunctionNewInterface>(this),state,t);

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
    
    void HyperplanValueFunction::prune(number t)
    {
        this->bounded_prune(t);
    }
    
    void HyperplanValueFunction::bounded_prune(number t)
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
                res << "\t\t\t" << plan << std::endl;
                res << "\t\t</plan>" << std::endl;
            }
            res << "\t</value>" << std::endl;
        }

        res << "</hyerplan_value_function>" << std::endl;
        return res.str();
    }
} // namespace sdm

namespace sdm
{
    PointSetValueFunction::PointSetValueFunction(std::shared_ptr<BackupInterface> backup,number horizon, std::shared_ptr<Initializer> initializer)
        : ValueFunction(backup,horizon), initializer_(initializer)
    {
        this->representation = std::vector<Container>(this->isInfiniteHorizon() ? 1 : this->horizon_ + 1, Container());
    }

    PointSetValueFunction::PointSetValueFunction(std::shared_ptr<BackupInterface> backup,number horizon, double default_value)
        : PointSetValueFunction(backup,horizon, std::make_shared<ValueInitializer>(default_value))
    {
    }

    void PointSetValueFunction::initialize()
    {
        this->initializer_->init(this->getptr());
    }

    void PointSetValueFunction::initialize(double default_value, number t)
    {
        this->representation[this->isInfiniteHorizon() ? 0 : t] = Container(default_value);
    }

    double PointSetValueFunction::getValueAt(const std::shared_ptr<State> &state, number t)
    {
        // double i_value = std::static_pointer_cast<BackupBase<double>>(this->backup_)->getMaxAt(std::shared_ptr<ValueFunction>(this),state,t).first;
        // // this->updateValueAt(state, t, i_value);
        // return i_value;

        if (t < this->getHorizon() && this->init_function_ != nullptr)
        {
            if ((this->representation[t].find(state) == this->representation[t].end()))
            {
                double i_value = std::static_pointer_cast<BackupBase<double>>(this->backup_)->getMaxAt(this->getptr(),state,t).first;
                this->updateValueAt(state, t, i_value);
                return i_value;
            }
        }
        return this->representation[this->isInfiniteHorizon() ? 0 : t].at(state);
    }

    void PointSetValueFunction::updateValueAt(const std::shared_ptr<State> &state, number t, double target)
    {
        this->representation[this->isInfiniteHorizon() ? 0 : t][state] = target;
    }

    void PointSetValueFunction::updateValueAt(const std::shared_ptr<State> &state, number t)
    {
        this->updateValueAt(state, t,std::static_pointer_cast<BackupBase<double>>(this->backup_)->backup(this->getptr(),state,t) );

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

    // void PointSetValueFunction::save(std::string filename)
    // {
    //     BoostSerializable<PointSetValueFunction>::save(filename);
    // }

    // void PointSetValueFunction::load(std::string filename)
    // {
    //     BoostSerializable<PointSetValueFunction>::load(filename);
    // }

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

    std::vector<std::shared_ptr<State>> PointSetValueFunction::getSupport(number t)
    {
        return this->representation[this->isInfiniteHorizon() ? 0 : t].getIndexes();
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
        auto pair_witness_ostate = std::static_pointer_cast<BackupBase<double>>(this->backup_)->getMaxAt(this->getptr(),state,t);
        // Faire passer le MaxAt avec le backup

        if (pair_witness_ostate.second == state)
        {
            return false;
        }
        else
        {
            return (pair_witness_ostate.first <= value + this->epsilon_prunning);
        }
    }

} // namespace sdm
#include <iomanip>

#include <sdm/utils/value_function/tabular_value_function.hpp>
#include <sdm/utils/value_function/backup/backup_base.hpp>

namespace sdm
{
    TabularValueFunction::TabularValueFunction(number horizon, const std::shared_ptr<Initializer> &initializer, const std::shared_ptr<BackupInterfaceForValueFunction> &backup, const std::shared_ptr<ActionVFInterface> &action_vf)
        : ValueFunction(horizon, initializer, backup,action_vf)
    {
        this->representation = std::vector<Container>(this->isInfiniteHorizon() ? 1 : this->horizon_ + 1, Container());
    }

    TabularValueFunction::TabularValueFunction(number horizon, double default_value, const std::shared_ptr<BackupInterfaceForValueFunction> &backup, const std::shared_ptr<ActionVFInterface> &action_vf)
        : TabularValueFunction(horizon, std::make_shared<ValueInitializer>(default_value), backup, action_vf)
    {
    }

    void TabularValueFunction::initialize()
    {
        this->initializer_->init(this->getptr());
    }

    void TabularValueFunction::initialize(double default_value, number t)
    {
        this->representation[this->isInfiniteHorizon() ? 0 : t] = Container(default_value);
    }

    double TabularValueFunction::getValueAt(const std::shared_ptr<State> &state, number t)
    {
        if (t < this->getHorizon() && this->init_function_ != nullptr)
        {
            if ((this->representation[t].find(state) == this->representation[t].end()))
            {
                double i_value = this->evaluate(state, t).second;
                this->updateValueAt(state, t, i_value);
                return i_value;
            }
        }
        return this->representation[this->isInfiniteHorizon() ? 0 : t].at(state);
    }

    Pair<std::shared_ptr<State>,double> TabularValueFunction::evaluate(const std::shared_ptr<State>& state, number t)
    {
        return std::make_pair(state,this->getInitFunction()->operator()(state,t));
    }

    void TabularValueFunction::updateValueAt(const std::shared_ptr<State> &state, number t, double target)
    {
        this->representation[this->isInfiniteHorizon() ? 0 : t][state] = target;
    }

    void TabularValueFunction::updateValueAt(const std::shared_ptr<State> &state, number t)
    {
        this->updateValueAt(state, t, this->template backup<double>(state,this->getBestAction(state,t), t));
    }

    // void TabularValueFunction::save(std::string filename)
    // {
    //     BoostSerializable<TabularValueFunction>::save(filename);
    // }

    // void TabularValueFunction::load(std::string filename)
    // {
    //     BoostSerializable<TabularValueFunction>::load(filename);
    // }

    std::string TabularValueFunction::str() const
    {
        std::ostringstream res;

        res << "<tabular_value_function horizon=\"" << ((this->isInfiniteHorizon()) ? "inf" : std::to_string(this->getHorizon())) << "\">" << std::endl;
        for (std::size_t i = 0; i < this->representation.size(); i++)
        {
            res << "\t<value_function t=\"" << ((this->isInfiniteHorizon()) ? "all" : std::to_string(i)) << "\" default=\"" << this->representation[i].getDefault() << "\">" << std::endl;
            for (const auto &pair_state_val : this->representation[i])
            {
                res<< "\t adress "<<pair_state_val.first;
                res << "\t\t<value state=\"" << pair_state_val.first->str() << "\"> " << std::setprecision(config::VALUE_DECIMAL_PRINT) << std::fixed << pair_state_val.second << " </value>" << std::endl;
            }
            res << "\t</value_function>" << std::endl;
        }

        res << "</tabular_value_function>" << std::endl;
        return res.str();
    }

    std::vector<std::shared_ptr<State>> TabularValueFunction::getSupport(number t)
    {
        return this->representation[this->isInfiniteHorizon() ? 0 : t].getIndexes();
    }

    MappedVector<std::shared_ptr<State>, double> TabularValueFunction::getRepresentation(number t)
    {
        return this->representation[this->isInfiniteHorizon() ? 0 : t];
    }

} // namespace sdm
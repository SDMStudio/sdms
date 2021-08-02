#include <iomanip>

#include <sdm/utils/value_function/tabular_value_function.hpp>
#include <sdm/utils/value_function/backup/backup_base.hpp>

namespace sdm
{
    template <class Hash, class KeyEqual>
    BaseTabularValueFunction<Hash, KeyEqual>::BaseTabularValueFunction(number horizon, const std::shared_ptr<Initializer> &initializer, const std::shared_ptr<BackupInterfaceForValueFunction> &backup, const std::shared_ptr<ActionVFInterface> &action_vf)
        : ValueFunction(horizon, initializer, backup, action_vf)
    {
        this->representation = std::vector<Container>(this->isInfiniteHorizon() ? 1 : this->horizon_ + 1, Container());
    }

    template <class Hash, class KeyEqual>
    BaseTabularValueFunction<Hash, KeyEqual>::BaseTabularValueFunction(number horizon, double default_value, const std::shared_ptr<BackupInterfaceForValueFunction> &backup, const std::shared_ptr<ActionVFInterface> &action_vf)
        : BaseTabularValueFunction(horizon, std::make_shared<ValueInitializer>(default_value), backup, action_vf)
    {
    }

    template <class Hash, class KeyEqual>
    void BaseTabularValueFunction<Hash, KeyEqual>::initialize()
    {
        this->initializer_->init(this->getptr());
    }

    template <class Hash, class KeyEqual>
    void BaseTabularValueFunction<Hash, KeyEqual>::initialize(double default_value, number t)
    {
        this->representation[this->isInfiniteHorizon() ? 0 : t] = Container(default_value);
    }

    template <class Hash, class KeyEqual>
    double BaseTabularValueFunction<Hash, KeyEqual>::getValueAt(const std::shared_ptr<State> &state, number t)
    {
        if (t < this->getHorizon() && this->init_function_ != nullptr)
        {
            if ((this->representation[t].find(state) == this->representation[t].end()))
            {
                double i_value = this->evaluate(state, t).second;
                // this->updateValueAt(state, t, i_value);
                return i_value;
            }
        }
        return this->representation[this->isInfiniteHorizon() ? 0 : t].at(state);
    }

    template <class Hash, class KeyEqual>
    Pair<std::shared_ptr<State>, double> BaseTabularValueFunction<Hash, KeyEqual>::evaluate(const std::shared_ptr<State> &state, number t)
    {
        return std::make_pair(state, this->getInitFunction()->operator()(state, t));
    }

    template <class Hash, class KeyEqual>
    void BaseTabularValueFunction<Hash, KeyEqual>::updateValueAt(const std::shared_ptr<State> &state, number t, double target)
    {
        this->representation[this->isInfiniteHorizon() ? 0 : t][state] = target;
    }

    template <class Hash, class KeyEqual>
    void BaseTabularValueFunction<Hash, KeyEqual>::updateValueAt(const std::shared_ptr<State> &state, number t)
    {
        auto [best_action, tmp] = this->getBestActionAndValue(state, t);
        this->updateValueAt(state, t, tmp);
    }

    template <class Hash, class KeyEqual>
    void BaseTabularValueFunction<Hash, KeyEqual>::updateValueAt(const std::shared_ptr<State> &state, const std::shared_ptr<Action>& action, number t)
    {
        auto value = this->template backup<double>(state, action, t);
        this->updateValueAt(state, t, value);
    }

    template <class Hash, class KeyEqual>
    size_t BaseTabularValueFunction<Hash, KeyEqual>::getSize(number t) const
    {
        return this->representation[this->isInfiniteHorizon() ? 0 : t].size();
    }

    // template <class Hash, class KeyEqual>
    // void BaseTabularValueFunction<Hash, KeyEqual>::save(std::string filename)
    // {
    //     BoostSerializable<BaseTabularValueFunction>::save(filename);
    // }

    // template <class Hash, class KeyEqual>
    // void BaseTabularValueFunction<Hash, KeyEqual>::load(std::string filename)
    // {
    //     BoostSerializable<BaseTabularValueFunction>::load(filename);
    // }

    template <class Hash, class KeyEqual>
    std::string BaseTabularValueFunction<Hash, KeyEqual>::str() const
    {
        std::ostringstream res;

        res << "<tabular_value_function horizon=\"" << ((this->isInfiniteHorizon()) ? "inf" : std::to_string(this->getHorizon())) << "\">" << std::endl;
        for (std::size_t i = 0; i < this->representation.size(); i++)
        {
            res << "\t<value_function t=\"" << ((this->isInfiniteHorizon()) ? "all" : std::to_string(i)) << "\" default=\"" << this->representation[i].getDefault() << "\">" << std::endl;
            for (const auto &pair_state_val : this->representation[i])
            {
                res << "\t\t<value>\n";
                tools::indentedOutput(res, pair_state_val.first->str().c_str(), 3);
                res << std::endl
                    << "\t\t\t" << std::setprecision(config::VALUE_DECIMAL_PRINT) << std::fixed << pair_state_val.second << std::endl;
                res << "\t\t</value>" << std::endl;
            }
            res << "\t</value_function>" << std::endl;
        }

        res << "</tabular_value_function>" << std::endl;
        return res.str();
    }

    template <class Hash, class KeyEqual>
    std::vector<std::shared_ptr<State>> BaseTabularValueFunction<Hash, KeyEqual>::getSupport(number t)
    {
        return this->representation[this->isInfiniteHorizon() ? 0 : t].getIndexes();
    }

    template <class Hash, class KeyEqual>
    typename BaseTabularValueFunction<Hash, KeyEqual>::Container BaseTabularValueFunction<Hash, KeyEqual>::getRepresentation(number t)
    {
        return this->representation[this->isInfiniteHorizon() ? 0 : t];
    }
    template <class Hash, class KeyEqual>
    void BaseTabularValueFunction<Hash, KeyEqual>::do_pruning(number ){}

} // namespace sdm
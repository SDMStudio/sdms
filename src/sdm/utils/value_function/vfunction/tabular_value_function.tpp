#include <iomanip>

#include <sdm/types.hpp>
#include <sdm/utils/value_function/initializer/initializer.hpp>
#include <sdm/utils/value_function/vfunction/tabular_value_function.hpp>

#include <sdm/utils/value_function/action_selection/exhaustive_action_selection.hpp>
#include <sdm/utils/value_function/initializer/initializer.hpp>

namespace sdm
{
    template <class Hash, class KeyEqual>
    BaseTabularValueFunction<Hash, KeyEqual>::BaseTabularValueFunction(const std::shared_ptr<SolvableByDP> &world,
                                                                       const std::shared_ptr<Initializer> &initializer,
                                                                       const std::shared_ptr<ActionSelectionInterface> &action_selection,
                                                                       Config config)
        : ValueFunctionInterface(world, initializer, action_selection),
          ValueFunction(world, initializer, action_selection),
          TabularValueFunctionInterface(world, initializer, action_selection)
    {
        this->representation = std::vector<Container>(this->isInfiniteHorizon() ? 1 : world->getHorizon() + 1, Container());
    }

    template <class Hash, class KeyEqual>
    BaseTabularValueFunction<Hash, KeyEqual>::BaseTabularValueFunction(const BaseTabularValueFunction &copy)
        : ValueFunctionInterface(copy.world_, copy.initializer_, copy.action_selection_),
          ValueFunction(copy),
          TabularValueFunctionInterface(copy.world_, copy.initializer_, copy.action_selection_),
          representation(copy.representation) {}

    template <class Hash, class KeyEqual>
    void BaseTabularValueFunction<Hash, KeyEqual>::initialize()
    {
        this->getInitializer()->init(this->getptr());
    }

    template <class Hash, class KeyEqual>
    void BaseTabularValueFunction<Hash, KeyEqual>::initialize(double default_value, number t)
    {
        this->representation[this->isInfiniteHorizon() ? 0 : t] = Container(default_value);
    }

    template <class Hash, class KeyEqual>
    double BaseTabularValueFunction<Hash, KeyEqual>::getValueAt(const std::shared_ptr<State> &state, number t)
    {
        if (this->init_function_ != nullptr && (t < this->getHorizon() || this->isInfiniteHorizon()))
        {
            if ((this->representation[this->isInfiniteHorizon() ? 0 : t].find(state) == this->representation[this->isInfiniteHorizon() ? 0 : t].end()))
            {
                return this->getInitFunction()->operator()(state, t);
            }
        }
        double value = this->representation[this->isInfiniteHorizon() ? 0 : t].at(state);
        return value;
    }

    template <class Hash, class KeyEqual>
    Pair<std::shared_ptr<State>, double> BaseTabularValueFunction<Hash, KeyEqual>::evaluate(const std::shared_ptr<State> &state, number t)
    {
        return std::make_pair(state, this->getInitFunction()->operator()(state, t));
    }

    template <class Hash, class KeyEqual>
    void BaseTabularValueFunction<Hash, KeyEqual>::setValueAt(const std::shared_ptr<State> &state, double new_value, number t)
    {
        this->representation[this->isInfiniteHorizon() ? 0 : t][state] = new_value;
    }

    template <class Hash, class KeyEqual>
    size_t BaseTabularValueFunction<Hash, KeyEqual>::getSize(number t) const
    {
        return this->representation[this->isInfiniteHorizon() ? 0 : t].size();
    }

    template <class Hash, class KeyEqual>
    void BaseTabularValueFunction<Hash, KeyEqual>::save(std::string filename)
    {
        BoostSerializable<BaseTabularValueFunction>::save(filename);
    }

    template <class Hash, class KeyEqual>
    void BaseTabularValueFunction<Hash, KeyEqual>::load(std::string filename)
    {
        BoostSerializable<BaseTabularValueFunction>::load(filename);
    }

    template <class Hash, class KeyEqual>
    std::shared_ptr<ValueFunctionInterface> BaseTabularValueFunction<Hash, KeyEqual>::copy()
    {
        auto casted_value = std::dynamic_pointer_cast<BaseTabularValueFunction<Hash, KeyEqual>>(this->getptr());
        return std::make_shared<BaseTabularValueFunction<Hash, KeyEqual>>(*casted_value);
    }

    template <class Hash, class KeyEqual>
    std::string BaseTabularValueFunction<Hash, KeyEqual>::str() const
    {
        std::ostringstream res;

        res << "<tabular_value_function horizon=\"" << ((this->isInfiniteHorizon()) ? "inf" : std::to_string(this->getHorizon())) << "\">" << std::endl;
        for (std::size_t i = 0; i < this->representation.size(); i++)
        {
            res << "\t<value_function t=\"" << ((this->isInfiniteHorizon()) ? "all" : std::to_string(i)) << "\" default=\"" << this->representation.at(i).getDefault() << "\">" << std::endl;
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
    typename BaseTabularValueFunction<Hash, KeyEqual>::Container &BaseTabularValueFunction<Hash, KeyEqual>::getRepresentation(number t)
    {
        return this->representation[this->isInfiniteHorizon() ? 0 : t];
    }

    template <class Hash, class KeyEqual>
    void BaseTabularValueFunction<Hash, KeyEqual>::do_pruning(number) {}
} // namespace sdm
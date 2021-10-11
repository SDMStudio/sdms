#include <sdm/utils/value_function/value_function.hpp>
#include <sdm/utils/linear_algebra/mapped_vector.hpp>
#include <sdm/utils/linear_algebra/vector_interface.hpp>

namespace sdm
{
    ValueFunction::ValueFunction(number horizon, const std::shared_ptr<Initializer> &initializer, const std::shared_ptr<BackupInterfaceForValueFunction> &backup, const std::shared_ptr<ActionVFInterface> &action)
        : BaseValueFunction(horizon), backup_(backup), action_(action), initializer_(initializer)
    {
    }

    ValueFunction::ValueFunction(const ValueFunction &copy) : BaseValueFunction(copy), backup_(copy.backup_), init_function_(copy.init_function_), action_(copy.action_), initializer_(copy.initializer_)
    {
    }

    std::shared_ptr<BinaryFunction<std::shared_ptr<State>, number, double>> ValueFunction::getInitFunction()
    {
        return this->init_function_;
    }

    double ValueFunction::operator()(const std::shared_ptr<State> &state, const number &t)
    {
        return this->getValueAt(state, t);
    }

    void ValueFunction::initialize(const std::shared_ptr<BinaryFunction<std::shared_ptr<State>, number, double>> &init_function)
    {
        this->init_function_ = init_function;
    }

    std::shared_ptr<ValueFunction> ValueFunction::getptr()
    {
        return std::static_pointer_cast<ValueFunction>(this->shared_from_this());
    }

    std::shared_ptr<Action> ValueFunction::getBestAction(const std::shared_ptr<State> &state, number t)
    {
        return this->getBestActionAndValue(state, t).first;
    }

    Pair<std::shared_ptr<Action>, double> ValueFunction::getBestActionAndValue(const std::shared_ptr<State> &state, number t)
    {
        auto pair_action_value = this->action_->selectBestAction(this->getptr(), state, t);

        return pair_action_value;
    }

    size_t ValueFunction::getSize() const
    {
        size_t size_total = 0;
        for (number t = 0; t < this->getHorizon(); t++)
        {
            size_total += this->getSize(t);
        }
        return size_total;
    }

} // namespace sdm
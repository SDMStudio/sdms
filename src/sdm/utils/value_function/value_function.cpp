#include <sdm/utils/value_function/value_function.hpp>
#include <sdm/utils/linear_algebra/mapped_vector.hpp>
#include <sdm/utils/linear_algebra/vector_interface.hpp>

namespace sdm
{
    ValueFunction::ValueFunction(number horizon, const std::shared_ptr<Initializer> &initializer, const std::shared_ptr<BackupInterfaceForValueFunction> &backup, const std::shared_ptr<ActionVFInterface> &action)
        : BaseValueFunction(horizon), backup_(backup), action_(action), initializer_(initializer)
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

    // std::shared_ptr<VectorInterface<std::shared_ptr<Action>, double>> ValueFunction::getQValueAt(const std::shared_ptr<State> &state, number t)
    // {
    //     // Compute Q(s,*)
    //     std::shared_ptr<MappedVector<std::shared_ptr<Action>, double>> q_s = std::make_shared<MappedVector<std::shared_ptr<Action>, double>>();
    //     for (const auto &action : *this->getWorld()->getActionSpaceAt(state, t))
    //     {
    //         auto casted_action = std::static_pointer_cast<Action>(action);
    //         (*q_s)[casted_action] = this->getQValueAt(state, casted_action, t);
    //     }
    //     return q_s;
    // }

    // double ValueFunction::getQValueAt(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t)
    // {
    //     // implement bellman operator
    //     return this->getWorld()->getReward(state, action, t) + this->getWorld()->getDiscount(t) * this->getWorld()->getExpectedNextValue(this->getptr(), state, action, t);
    // }

    // std::shared_ptr<Action> ValueFunction::getBestAction(const std::shared_ptr<State> &state, number t)
    // {
    //     // Get the best action (i.e. the action that maximizes the q value function)
    //     return this->backup_->getBestAction(this->getptr(), state, t);
    // }

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
        return this->action_->selectBestAction(this->getptr(), state, t);
    }
} // namespace sdm
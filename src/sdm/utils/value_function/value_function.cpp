#include <sdm/utils/linear_algebra/vector_interface.hpp>
#include <sdm/utils/linear_algebra/mapped_vector.hpp>
#include <sdm/utils/value_function/value_function.hpp>

namespace sdm
{
    ValueFunction::ValueFunction(std::shared_ptr<BackupInterface> backup,number horizon) : BaseValueFunction(horizon), backup_(backup)
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

    std::shared_ptr<Action> ValueFunction::getBestAction(const std::shared_ptr<State> &state, number t)
    {
        // Get the best action (i.e. the action that maximizes the q value function)
        return this->backup_->getBestAction(std::shared_ptr<ValueFunction>(this),state,t);
    }

    void ValueFunction::initialize(std::shared_ptr<BinaryFunction<std::shared_ptr<State>, number, double>> init_function)
    {
        this->init_function_ = init_function;
    }

    // std::shared_ptr<SolvableByHSVI> ValueFunction::getWorld()
    // {
    //     return this->problem_;
    // }
    
    std::shared_ptr<ValueFunction> ValueFunction::getptr()
    {
        return std::static_pointer_cast<ValueFunction>(this->shared_from_this());
    }

} // namespace sdm
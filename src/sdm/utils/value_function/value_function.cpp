#include <sdm/utils/value_function/value_function.hpp>
#include <sdm/utils/linear_algebra/mapped_vector.hpp>
#include <sdm/utils/linear_algebra/vector_interface.hpp>

namespace sdm
{
    ValueFunction::ValueFunction(number horizon, const std::shared_ptr<Initializer> &initializer, const std::shared_ptr<BackupInterfaceForValueFunction> &backup, const std::shared_ptr<ActionVFInterface> &action)
        : BaseValueFunction(horizon), backup_(backup), action_(action), initializer_(initializer)
    {
    }

    ValueFunction::ValueFunction(const ValueFunction &copy) : BaseValueFunction(copy), init_function_(copy.init_function_), backup_(copy.backup_), action_(copy.action_), initializer_(copy.initializer_)
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
#ifdef LOGTIME
        std::chrono::high_resolution_clock::time_point time_start = std::chrono::high_resolution_clock::now();
#endif

        auto pair_action_value = this->action_->selectBestAction(this->getptr(), state, t);

#ifdef LOGTIME
        this->updateTime(time_start, "Best Action");
#endif

        return pair_action_value;
    }

#ifdef LOGTIME
    void ValueFunction::updateTime(std::chrono::high_resolution_clock::time_point start_time, std::string information)
    {
        if (information == "backup")
        {
            this->total_time_update_backup += std::Performance::computeTime(start_time);
        }
        else if (information == "Best Action")
        {
            this->total_time_update_best_action += std::Performance::computeTime(start_time);
        }
        else if (information == "Exist")
        {
            this->total_time_exist += std::Performance::computeTime(start_time);
        }
        else if (information == "Evaluate")
        {
            this->total_time_evaluate += std::Performance::computeTime(start_time);
        }
        else if (information == "GetValueAt")
        {
            this->time_get_value_at += std::Performance::computeTime(start_time);
        }
        else if (information == "UpdateValue")
        {
            this->time_update_value += std::Performance::computeTime(start_time);
        }
        else if (information == "Pruning")
        {
            this->time_pruning += std::Performance::computeTime(start_time);
        }
    }

    double ValueFunction::getTime(std::string information)
    {
        if (information == "backup")
        {
            return this->total_time_update_backup;
        }
        else if (information == "Best Action")
        {
            return this->total_time_update_best_action;
        }
        else if (information == "Exist")
        {
            return this->total_time_exist;
        }
        else if (information == "Evaluate")
        {
            return this->total_time_evaluate;
        }
        else if (information == "GetValueAt")
        {
            return this->time_get_value_at;
        }
        else if (information == "UpdateValue")
        {
            return this->time_update_value;
        }
        else if (information == "Pruning")
        {
            return this->time_pruning;
        }
        else
        {
            return -1;
        }
    }

#endif

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
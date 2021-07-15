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
        #ifdef LOGTIME
            this->StartTime();
        #endif

        auto action = this->action_->selectBestAction(this->getptr(), state, t);

        #ifdef LOGTIME
            this->updateTime("Best Action");
        #endif

        return action;
    }
    
    #ifdef LOGTIME
        void ValueFunction::StartTime()
        {
            this->time_start = clock();
        }

        void ValueFunction::updateTime(std::string information)
        {
            if(information == "backup")
            {
                this->total_time_update_backup += (float)(clock() - this->time_start)/ CLOCKS_PER_SEC;
            }
            else if(information == "Best Action")
            {
                this->total_time_update_best_action += (float)(clock() - this->time_start)/ CLOCKS_PER_SEC;
            }
            else if(information == "Exist")
            {
                this->total_time_exist += (float)(clock() - this->time_start )/ CLOCKS_PER_SEC;
            }            
            else if(information == "Evaluate")
            {
                this->total_time_evaluate += (float)(clock() - this->time_start)/ CLOCKS_PER_SEC;
            }
        }

        void ValueFunction::printTime()
        {
            std::cout<<"\t Total Time Backup :"<<this->total_time_update_backup<<std::endl;
            std::cout<<"\t Total Time Best Action :"<<this->total_time_update_best_action<<std::endl;
            std::cout<<"\t Total Time Exist :"<<this->total_time_exist<<std::endl;
            std::cout<<"\t Total Time Evaluate : "<<this->total_time_evaluate<<std::endl;
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
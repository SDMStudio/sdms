#pragma once

#include <sdm/types.hpp>
#include <sdm/utils/value_function/value_function_interface.hpp>

namespace sdm
{

    class TabularValueFunctionInterface;
    class PWLCValueFunctionInterface;

    namespace update
    {
        /**
         * @brief This interface is the interface that is common to all update operators.
         *
         * Any class inheriting from this interface can be used to update a value function.
         *
         */
        class UpdateOperatorInterface
        {
        public:
            /**
             * @brief Update the value function.
             *
             * @param t the time step
             */
            virtual void update(const std::shared_ptr<State> &state, number t) = 0;
        };

        /**
         * @brief This class is templated to allow user to specify the structure of the value
         * function they managed.
         *
         * @tparam TValueFunction the type of value function
         */
        template <class TValueFunction>
        class UpdateOperator : public UpdateOperatorInterface
        {
        public:
            UpdateOperator() {}
            UpdateOperator(const std::shared_ptr<ValueFunctionInterface> &value_function)
            {
                if (auto derived = std::dynamic_pointer_cast<TValueFunction>(value_function))
                {
                    this->value_function = derived;
                }
                else
                {
                    throw sdm::exception::TypeError("Cannot instanciate UpdateOperator<T> with value function that does not derive from T.");
                }
            }
            virtual ~UpdateOperator() {}


            /**
             * @brief Update the value function.
             *
             * @param t the time step
             */
            virtual void update(const std::shared_ptr<State> &state, number t) = 0;

            /**
             * @brief Get the updatable value function 
             * 
             * @return the value function
             */
            inline std::shared_ptr<TValueFunction> getValueFunction() const
            {
                return this->value_function;
            }

            /**
             * @brief Set the updatable value function
             * 
             * @param value_function the value function
             */
            void setValueFunction(const std::shared_ptr<TValueFunction> &value_function) const
            {
                this->value_function = value_function;
            }

            /**
             * @brief Get the world
             * 
             * @return the world 
             */
            inline std::shared_ptr<SolvableByDP> getWorld() const
            {
                return this->getValueFunction()->getWorld();
            }

        protected:
            std::shared_ptr<TValueFunction> value_function;
        };

        using TabularUpdateOperator = UpdateOperator<TabularValueFunctionInterface>;
        using PWLCUpdateOperator = UpdateOperator<PWLCValueFunctionInterface>;
    }
}
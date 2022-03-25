#pragma once

#include <sdm/types.hpp>
#include <sdm/utils/rl/experience_memory.hpp>
#include <sdm/utils/value_function/value_function_interface.hpp>

namespace sdm
{

    class TabularQValueFunctionInterface;
    class PWLCValueFunctionInterface;

    namespace update
    {
        /**
         * @brief This interface is the interface that is common to all update operators.
         *
         * Any class inheriting from this interface can be used to update a value function.
         *
         */
        class QUpdateOperatorInterface
        {
        public:
            /**
             * @brief Update the value function.
             *
             * @param t the time step
             */
            virtual void update(double learning_rate, number t) = 0;
        };

        /**
         * @brief Update operator for reinforcement learning.
         *
         * @tparam TQValueFunction the type of Q-value function
         */
        template <class TQValueFunction>
        class QUpdateOperator : public QUpdateOperatorInterface
        {
        public:
            /**
             * @brief Construct an update operator for RL.
             *
             * @param qvalue_function the qvalue function
             */
            QUpdateOperator(std::shared_ptr<ExperienceMemory> experience_memory,
                            std::shared_ptr<ValueFunctionInterface> q_value,
                            std::shared_ptr<ValueFunctionInterface> target_q_value)
                : experience_memory(experience_memory)
            {
                auto derived = std::dynamic_pointer_cast<TQValueFunction>(q_value);
                auto derived_target = std::dynamic_pointer_cast<TQValueFunction>(target_q_value);
                if (derived && derived_target)
                {
                    this->q_value = derived;
                    this->target_q_value = derived_target;
                }
                else
                {
                    throw sdm::exception::TypeError("Cannot instanciate QUpdateOperator<T> with q-value function that does not derive from T.");
                }
            }

            virtual ~QUpdateOperator() {}

            /**
             * @brief Update the value function.
             *
             * @param t the time step
             */
            virtual void update(double learning_rate, number t) = 0;

            /**
             * @brief Get the updatable q-value function
             *
             *
             * @return the q-value function
             */
            inline std::shared_ptr<TQValueFunction> getQValueFunction() const
            {
                return this->q_value.lock();
            }

            /**
             * @brief Set the updatable q-value function
             *
             * @param q_value the q-value function
             */
            void setQValueFunction(const std::shared_ptr<TQValueFunction> &q_value) const
            {
                this->q_value = q_value;
            }

            /**
             * @brief Get the world
             *
             * @return the world
             */
            inline std::shared_ptr<SolvableByDP> getWorld() const
            {
                return this->getQValueFunction()->getWorld();
            }

        protected:
            /**
             * @brief Experience memory.
             */
            std::shared_ptr<ExperienceMemory> experience_memory;

            /**
             * @brief the Q-value function
             */
            std::weak_ptr<TQValueFunction> q_value, target_q_value;
        };

        using TabularQUpdateOperator = QUpdateOperator<TabularQValueFunctionInterface>;
        using PWLCQUpdateOperator = QUpdateOperator<PWLCValueFunctionInterface>;

    } // namespace update
} // namespace sdm

#pragma once

#include <sdm/types.hpp>

namespace sdm
{
    namespace update
    {
        /**
         * @brief This interface is the interface that is common to all update operators.
         *
         * Any class inheriting from this interface can be used to update a value function.
         * The value function is
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
            virtual void update(number t) = 0;
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
            QUpdateOperator(const std::shared_ptr<TQValueFunction> &value_function);

            /**
             * @brief Update the value function.
             *
             * @param t the time step
             */
            virtual void update(number t) = 0;

        protected:
            /**
             * @brief the Q-value function
             */
            std::shared_ptr<TQValueFunction> qvalue_function;
        };

        using QPWLCQUpdateOperator = QUpdateOperator<PWLCQValueFunction>;
        using QTabularQUpdateOperator = QUpdateOperator<TabularQValueFunction>;

    } // namespace update
} // namespace sdm

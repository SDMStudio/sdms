/**
 * @file initializer.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief File that contains definition of different initializers.
 * @version 1.0
 * @date 24/03/2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

#include <math.h>
#include <sdm/types.hpp>
#include <sdm/utils/value_function/value_function.hpp>
#include <sdm/utils/value_function/qvalue_function.hpp>
#include <sdm/utils/value_function/qvalue_function.hpp>

namespace sdm
{
    /**
     * @brief Abstract class for initializer. 
     * 
     * @tparam TState the state type
     * @tparam TAction the action type
     */
    class Initializer
    {
    public:
        virtual void init(std::shared_ptr<ValueFunction> vf) = 0;
        virtual ~Initializer() {}
    };

    /**
     * @brief Abstract class for initializer. 
     * 
     * @tparam TState the state type
     * @tparam TAction the action type
     */
    class QInitializer
    {
    public:
        virtual void init(std::shared_ptr<QValueFunction> vf) = 0;
        virtual ~QInitializer() {}
    };

    /**
     * @brief  This initializer initializes a value function to a constant value.
     * 
     * @tparam TState the state type
     * @tparam TAction the action type
     */
    class ValueInitializer : public Initializer, public QInitializer
    {
    protected:
        double value;

    public:
        ValueInitializer(double v);

        void initBase(std::shared_ptr<BaseValueFunction> vf);

        void init(std::shared_ptr<ValueFunction> vf);

        void init(std::shared_ptr<QValueFunction> vf);
    };

    /**
     * @brief This initializer initializes a value function to zero.
     * 
     * @tparam TState the state type
     * @tparam TAction the action type
     */
    class ZeroInitializer : public ValueInitializer
    {
    public:
        ZeroInitializer();
    };

    /**
     * @brief This initializer initializes a value function to the estimation of the value if we get a constant reward at every timestep.
     * 
     * @tparam TState the state type
     * @tparam TAction the action type
     */
    class BoundInitializer : public Initializer
    {
    protected:
        double value_;
        double (MDPInterface::*callback_value)(number) const = nullptr;
        std::shared_ptr<SolvableByHSVI> world_;

    public:
        BoundInitializer();
        BoundInitializer(std::shared_ptr<SolvableByHSVI> world,double value);

        void init(std::shared_ptr<ValueFunction> vf);

        double getValue(std::shared_ptr<ValueFunction> vf, number t);

        double computeValueInfiniteHorizon(std::shared_ptr<ValueFunction> vf);
    };

    /**
     * @brief This initializer initializes a value function to the worst value. This is a pessimistic initialization.
     * 
     * @tparam TState the state type
     * @tparam TAction the action type
     */
    class MinInitializer : public BoundInitializer
    {
    public:
        MinInitializer(std::shared_ptr<SolvableByHSVI> world);

        void init(std::shared_ptr<ValueFunction> vf);
    };

    /**
     * @brief This initializer initializes a value function to the best value. This is an optimistic initialization.
     * 
     * @tparam TState the state type
     * @tparam TAction the action type
     */
    class MaxInitializer : public BoundInitializer
    {
    public:
        MaxInitializer(std::shared_ptr<SolvableByHSVI> world);

        void init(std::shared_ptr<ValueFunction> vf);
    };

    /**
     * @brief This initializer calculates the initial lower bound $\bar{V}_0$ using the blind  policy method [Hauskrecht, 1997]. 
     * Trey Smith and Reid Simmons used this initialization procedure in https://arxiv.org/pdf/1207.4166.pdf . 
     * 
     */
    class BlindInitializer : public BoundInitializer
    {
    public:
        BlindInitializer(std::shared_ptr<SolvableByHSVI> world);

        void init(std::shared_ptr<ValueFunction> vf);
    };

} // namespace sdm
#include <sdm/utils/value_function/initializer.tpp>


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
#include <sdm/world/base/mdp_interface.hpp>
// #include <sdm/utils/value_function/value_function_interface.hpp>
// #include <sdm/utils/value_function/value_function.hpp>
// #include <sdm/utils/value_function/qvalue_function.hpp>

namespace sdm
{
    class SolvableByHSVI;
    class ValueFunctionInterface;

    /**
     * @brief Abstract class for state value function initializer. 
     * 
     */
    class Initializer
    {
    public:
        virtual ~Initializer() {}
        virtual void init(std::shared_ptr<ValueFunctionInterface> vf) = 0;
    };
    
    /**
     * @brief This initializer initializes a value function to a constant value.
     * 
     */
    class ValueInitializer : public Initializer
    {
    protected:
        double value;

    public:
        ValueInitializer(double v);
        void init(std::shared_ptr<ValueFunctionInterface> vf);
    };

    /**
     * @brief This initializer initializes a value function to zero.
     * 
     */
    class ZeroInitializer : public ValueInitializer
    {
    public:
        ZeroInitializer(std::shared_ptr<SolvableByHSVI> = nullptr) : ValueInitializer(0) {}
    };

    /**
     * @brief This initializer initializes a value function to the estimation of the value if we get a constant reward at every timestep.
     * 
     */
    class BoundInitializer : public Initializer
    {
    protected:
        double value_;
        double (MDPInterface::*callback_value)(number) const = nullptr;
        std::shared_ptr<SolvableByHSVI> world_;

    public:
        BoundInitializer();
        BoundInitializer(std::shared_ptr<SolvableByHSVI> world, double value);

        void init(std::shared_ptr<ValueFunctionInterface> vf);
        double getValue(std::shared_ptr<ValueFunctionInterface> vf, number t);
        double computeValueInfiniteHorizon(std::shared_ptr<ValueFunctionInterface> vf);
    };

    /**
     * @brief This initializer initializes a value function to the worst value. This is a pessimistic initialization.
     * 
     */
    class MinInitializer : public BoundInitializer
    {
    public:
        MinInitializer(std::shared_ptr<SolvableByHSVI> world);

        void init(std::shared_ptr<ValueFunctionInterface> vf);
    };

    /**
     * @brief This initializer initializes a value function to the best value. This is an optimistic initialization.
     * 
     */
    class MaxInitializer : public BoundInitializer
    {
    public:
        MaxInitializer(std::shared_ptr<SolvableByHSVI> world);

        void init(std::shared_ptr<ValueFunctionInterface> vf);
    };

    /**
     * @brief This initializer calculates the initial lower bound $\bar{V}_0$ using the blind  policy method [Hauskrecht, 1997]. 
     * 
     * Trey Smith and Reid Simmons used this initialization procedure in https://arxiv.org/pdf/1207.4166.pdf . 
     * 
     */
    class BlindInitializer : public BoundInitializer
    {
    public:
        BlindInitializer(std::shared_ptr<SolvableByHSVI> world);

        void init(std::shared_ptr<ValueFunctionInterface> vf);
    };

} // namespace sdm

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
#include <sdm/utils/value_function/base_value_function.hpp>
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
        ValueInitializer(double v) : value(v)
        {
        }

        void initBase(std::shared_ptr<BaseValueFunction> vf)
        {
            if (vf->getHorizon() < 1)
            {
                vf->initialize(this->value);
            }
            else
            {
                for (number t = 0; t < vf->getHorizon(); t++)
                {
                    vf->initialize(this->value, t);
                }

                vf->initialize(0, vf->getHorizon());
            }
        }

        void init(std::shared_ptr<ValueFunction> vf)
        {
            this->initBase(vf);
        }

        void init(std::shared_ptr<QValueFunction> vf)
        {
            this->initBase(vf);
        }
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
        ZeroInitializer() : ValueInitializer(0)
        {
        }
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

    public:
        BoundInitializer() {}
        BoundInitializer(double value) : value_(value) {}

        void init(std::shared_ptr<ValueFunction> vf)
        {

            if (vf->isInfiniteHorizon())
            {
                vf->initialize(this->computeValueInfiniteHorizon(vf));
            }
            else
            {
                double tot = 0;
                vf->initialize(tot, vf->getHorizon());
                for (number t = vf->getHorizon(); t > 0; t--)
                {
                    // tot = this->getValue(vf, t) + vf->getWorld()->getUnderlyingProblem()->getDiscount(t) * tot;
                    vf->initialize(tot, t - 1);
                }
            }
        }

        double getValue(std::shared_ptr<ValueFunction> vf, number t)
        {
            return 0;
            // return (this->callback_value == nullptr) ? this->value_ : ((*vf->getWorld()->getUnderlyingProblem()).*callback_value)(t);
        }

        double computeValueInfiniteHorizon(std::shared_ptr<ValueFunction> vf)
        {
            // auto under_pb = vf->getWorld()->getUnderlyingProblem();
            // long l = log(1 - this->discount_) * this->error_ / this->reward_->getMaxReward();
            // number t = 0;
            // double value = this->getValue(vf, t), factor = 1.;
            // do
            // {
            //     factor *= under_pb->getDiscount(t);
            //     value += factor * this->getValue(vf, t + 1);
            //     t++;
            // } while (factor < 1.e-10);
            // value = floor(value) + (value > 0); // value = -2.99 --> floor(-2.99) + 0 = -3.0 and 2.99 --> floor(2.99) + 1 = 2 + 1 = 3.0
            // return value;
            return 0;
        }
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
        MinInitializer() { std::cout << "In MinInitalizer" << std::endl; }

        void init(std::shared_ptr<ValueFunction> vf)
        {
            //vf->getWorld()->getUnderlyingProblem()->getMinReward();
            this->callback_value = &MDPInterface::getMinReward;
            BoundInitializer::init(vf);
        }
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
        MaxInitializer() { std::cout << "In MaxInitalizer" << std::endl; }

        void init(std::shared_ptr<ValueFunction> vf)
        {
            // this->value_ = vf->getWorld()->getUnderlyingProblem()->getMaxReward();
            this->callback_value = &MDPInterface::getMaxReward;
            BoundInitializer::init(vf);
        }
    };

    /**
     * @brief This initializer calculates the initial lower bound $\bar{V}_0$ using the blind  policy method [Hauskrecht, 1997]. 
     * Trey Smith and Reid Simmons used this initialization procedure in https://arxiv.org/pdf/1207.4166.pdf . 
     * 
     */
    class BlindInitializer : public BoundInitializer
    {
    public:
        BlindInitializer() { std::cout << "In BlindInitalizer" << std::endl; }

        void init(std::shared_ptr<ValueFunction> vf)
        {
            // auto under_pb = vf->getWorld()->getUnderlyingProblem();
            // std::vector<double> ra, rt;

            // for (number t = 0; t < vf->getHorizon(); t++)
            // {
            //     ra.clear();
            //     for (auto &a : *under_pb->getActionSpace(t))
            //     {
            //         ra.push_back(std::numeric_limits<double>::max());
            //         for (auto &s : *under_pb->getStateSpace(t))
            //         {
            //             ra.back() = std::min(under_pb->getReward(std::static_pointer_cast<State>(s), std::static_pointer_cast<Action>(a), t), ra.back());
            //         }
            //     }
            //     rt.push_back(*std::max_element(ra.begin(), ra.end()));
            // }

            // this->value_ = *std::min_element(rt.begin(), rt.end());
            // BoundInitializer::init(vf);
        }
    };

} // namespace sdm

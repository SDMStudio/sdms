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
#include <sdm/utils/value_function/base_value_function.hpp>
#include <sdm/utils/value_function/qvalue_function.hpp>
#include <sdm/utils/value_function/qvalue_function.hpp>
#include <sdm/utils/value_function/state_2_occupancy_vf.hpp>

namespace sdm
{
    /**
     * @brief Abstract class for initializer. 
     * 
     * @tparam TState the state type
     * @tparam TAction the action type
     */
    template <typename TState, typename TAction>
    class Initializer
    {
    public:
        virtual void init(std::shared_ptr<ValueFunction<TState, TAction>> vf) = 0;
        virtual ~Initializer() {}
    };

    /**
     * @brief Abstract class for initializer. 
     * 
     * @tparam TState the state type
     * @tparam TAction the action type
     */
    template <typename TState, typename TAction>
    class QInitializer
    {
    public:
        virtual void init(std::shared_ptr<QValueFunction<TState, TAction>> vf) = 0;
        virtual ~QInitializer() {}
    };

    /**
     * @brief  This initializer initializes a value function to a constant value.
     * 
     * @tparam TState the state type
     * @tparam TAction the action type
     */
    template <typename TState, typename TAction>
    class ValueInitializer : public Initializer<TState, TAction>, public QInitializer<TState, TAction>
    {
    protected:
        double value;

    public:
        ValueInitializer(double v) : value(v)
        {
        }

        void initBase(std::shared_ptr<BaseValueFunction<TState, TAction>> vf)
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

        void init(std::shared_ptr<ValueFunction<TState, TAction>> vf)
        {
            this->initBase(vf);
        }

        void init(std::shared_ptr<QValueFunction<TState, TAction>> vf)
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
    template <typename TState, typename TAction>
    class ZeroInitializer : public ValueInitializer<TState, TAction>
    {
    public:
        ZeroInitializer() : ValueInitializer<TState, TAction>(0)
        {
        }
    };

    /**
     * @brief This initializer initializes a value function to the estimation of the value if if we get a constant reward at every timestep.
     * 
     * @tparam TState the state type
     * @tparam TAction the action type
     */
    template <typename TState, typename TAction>
    class BoundInitializer : public Initializer<TState, TAction>
    {
    protected:
        double value_;

    public:
        BoundInitializer() {}
        BoundInitializer(double value) : value_(value) {}

        void init(std::shared_ptr<ValueFunction<TState, TAction>> vf)
        {
            auto under_pb = vf->getWorld()->getUnderlyingProblem();

            if (vf->isInfiniteHorizon())
            {
                // long l = log(1 - this->discount_) * this->error_ / this->reward_->getMaxReward();
                assert(under_pb->getDiscount() < 1);
                double value;
                double factor = 0, comp = 0;
                number n = 0;
                do
                {
                    comp = factor;
                    factor += std::pow(under_pb->getDiscount(), n);
                    n++;
                } while ((factor - comp) > 0.0001);
                value = floor(this->value_ * factor) + 1;
                vf->initialize(value);
            }
            else
            {
                double tot = 0;
                vf->initialize(tot, vf->getHorizon());
                for (number t = vf->getHorizon(); t > 0; t--)
                {
                    tot = this->getValue(vf, t) + under_pb->getDiscount(t) * tot;
                    vf->initialize(tot, t - 1);
                }
            }
        }

        double getValue(std::shared_ptr<ValueFunction<TState, TAction>> vf, number t)
        {
            double value = 0;
            if (vf->getWorld()->isSerialized())
            {
                if (t % vf->getWorld()->getUnderlyingProblem()->getNumAgents() == 0)
                {
                    value = this->value_;
                }
            }
            else
            {
                value = this->value_;
            }
            return value;
        }
    };

    /**
     * @brief This initializer initializes a value function to the worst value. This is a pessimistic initialization.
     * 
     * @tparam TState the state type
     * @tparam TAction the action type
     */
    template <typename TState, typename TAction>
    class MinInitializer : public BoundInitializer<TState, TAction>
    {
    public:
        MinInitializer() { std::cout << "In MinInitalizer" << std::endl; }

        void init(std::shared_ptr<ValueFunction<TState, TAction>> vf)
        {
            this->value_ = vf->getWorld()->getUnderlyingProblem()->getReward()->getMinReward();
            BoundInitializer<TState, TAction>::init(vf);
        }
    };

    /**
     * @brief This initializer initializes a value function to the best value. This is an optimistic initialization.
     * 
     * @tparam TState the state type
     * @tparam TAction the action type
     */
    template <typename TState, typename TAction>
    class MaxInitializer : public BoundInitializer<TState, TAction>
    {
    public:
        MaxInitializer() { std::cout << "In MaxInitalizer" << std::endl; }

        void init(std::shared_ptr<ValueFunction<TState, TAction>> vf)
        {
            this->value_ = vf->getWorld()->getUnderlyingProblem()->getReward()->getMaxReward();
            BoundInitializer<TState, TAction>::init(vf);
        }
    };

    /**
     * @brief This initializer calculates the initial lower bound $\bar{V}_0$ using the blind  policy method [Hauskrecht, 1997]. 
     * Trey Smith and Reid Simmons used this initialization procedure in https://arxiv.org/pdf/1207.4166.pdf . 
     * 
     */
    template <typename TState, typename TAction>
    class BlindInitializer : public Initializer<TState, TAction>
    {
    public:
        BlindInitializer() { std::cout << "In BlindInitalizer" << std::endl; }

        void init(std::shared_ptr<ValueFunction<TState, TAction>> vf)
        {
            auto under_pb = vf->getWorld()->getUnderlyingProblem();
            std::vector<double> ra;

            for (auto &a : under_pb->getActionSpace()->getAll())
            {
                ra.push_back(std::numeric_limits<double>::max());
                for (auto &s : under_pb->getStateSpace()->getAll())
                {
                    ra.back() = std::min(under_pb->getReward(s, a), ra.back());
                }
            }
            if (vf->isInfiniteHorizon())
            {
                vf->initialize(*std::max_element(ra.begin(), ra.end()) / (1. - under_pb->getDiscount()));
            }
            else
            {
                double min_rsa = *std::max_element(ra.begin(), ra.end()), tot = 0;
                vf->initialize(tot, vf->getHorizon());
                for (number t = vf->getHorizon(); t > 0; t--)
                {
                    tot = min_rsa + under_pb->getDiscount(t) * tot;
                    vf->initialize(tot, t - 1);
                }
            }
        }
    };

} // namespace sdm

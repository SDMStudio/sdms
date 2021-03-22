#pragma once
#include <math.h>

#include <sdm/utils/value_function/value_function.hpp>

namespace sdm
{
    template <typename TState, typename TAction>
    class Initializer
    {
    public:
        virtual void init(ValueFunction<TState, TAction> *vf) = 0;
    };

    template <typename TState, typename TAction>
    class ValueInitializer : public Initializer<TState, TAction>
    {
    protected:
        double value;

    public:
        ValueInitializer(double v) : value(v)
        {
        }

        void init(ValueFunction<TState, TAction> *vf)
        {
            if (vf->getHorizon() < 1)
            {
                vf->initialize(this->value);
            }
            else
            {
                for (int t = 0; t < vf->getHorizon(); t++)
                {
                    vf->initialize(this->value, t);
                }
            }
        }
    };

    template <typename TState, typename TAction>
    class ZeroInitializer : public ValueInitializer<TState, TAction>
    {
    public:
        ZeroInitializer() : ValueInitializer<TState, TAction>(0)
        {
        }
    };

    template <typename TState, typename TAction>
    class BoundInitializer : public Initializer<TState, TAction>
    {
        double value_, discount_;
        int extensive_agent_;

    public:
        BoundInitializer(double value, double discount,int extensive_agent = 1)  : value_(value), discount_(discount),extensive_agent_(extensive_agent)
        {
        }

        void init(ValueFunction<TState, TAction> *vf)
        {
            if (vf->isInfiniteHorizon())
            {
                assert(this->discount_ < 1);
                double value;
                double factor = 0, comp = 0;
                int n = 0;
                do
                {
                    comp = factor;
                    factor += std::pow(this->discount_, n);
                    n++;
                } while ((factor - comp) > 0.0001);
                value = floor(this->value_ * factor) + 1;
                vf->initialize(value);
            }
            else
            {
                for (int t = 0; t < vf->getHorizon(); t++)
                {
                    if(extensive_agent_>1)
                    {
                        vf->initialize(this->value_ * ((vf->getHorizon() - t+1)/extensive_agent_), t);
                    }else
                    {
                        vf->initialize(this->value_ * (vf->getHorizon() - t), t);
                    }
                }
            }
        }
    };

    template <typename TState, typename TAction>
    class MinInitializer : public BoundInitializer<TState, TAction>
    {
    public:
        MinInitializer(double min_reward, double discount,int extensive_agent = 1) : BoundInitializer<TState, TAction>(min_reward, discount,extensive_agent)
        {
        }
    };

    template <typename TState, typename TAction>
    class MaxInitializer : public BoundInitializer<TState, TAction>
    {
    public:
        MaxInitializer(double max_reward, double discount,int extensive_agent = 1) : BoundInitializer<TState, TAction>(max_reward, discount,extensive_agent)
        {
        }
    };

    // template <typename TState, typename TAction>
    // class MDPInitializer : public Initializer<TState, TAction>
    // {
    // protected:
    //     std::string algo_name_;
    //     std::shared_ptr<DiscretePOMDP> problem_;
    //     double discount_;

    // public:
    //     MDPInitializer(std::string algo_name, std::shared_ptr<DiscretePOMDP> problem, double discount) : algo_name_(algo_name), problem_(problem), discount_(discount)
    //     {
    //     }

    //     void init(ValueFunction<TState, TAction> *vf)
    //     {
    //         auto algo = sdm::algo::make(algo_name, this->problem->toMDP());
    //         algo->do_solve();
    //         auto ubound = algo->getUpperBound();
    //     }
    // };
} // namespace sdm

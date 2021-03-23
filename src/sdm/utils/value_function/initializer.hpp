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

    public:
        BoundInitializer(double value, double discount) : value_(value), discount_(discount)
        {
        }

        void init(ValueFunction<TState, TAction> *vf)
        {
            if (vf->isInfiniteHorizon())
            {

                // long l = log(1 - this->discount_) * this->error_ / this->reward_->getMaxReward();

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
                    vf->initialize(this->value_ * (vf->getHorizon() - t), t);
                }
            }
        }
    };

    template <typename TState, typename TAction>
    class MinInitializer : public BoundInitializer<TState, TAction>
    {
    public:
        MinInitializer(double min_reward, double discount) : BoundInitializer<TState, TAction>(min_reward, discount)
        {
        }
    };

    template <typename TState, typename TAction>
    class MaxInitializer : public BoundInitializer<TState, TAction>
    {
    public:
        MaxInitializer(double max_reward, double discount) : BoundInitializer<TState, TAction>(max_reward, discount)
        {
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
        BlindInitializer()
        {
        }

        void init(ValueFunction<TState, TAction> *vf)
        {
            auto under_pb = vf->getWorld()->getUnderlyingProblem();

            if (vf->isInfiniteHorizon())
            {
                std::vector<double> ra;
                for (auto &a : under_pb->getActionSpace()->getAll())
                {
                    ra.push_back(std::numeric_limits<double>::max());
                    for (auto &s : under_pb->getStateSpace()->getAll())
                    {
                        ra.back() = std::min(under_pb->getReward(s, a)/ (1. - under_pb->getDiscount()), ra.back());
                    }
                }
                vf->initialize(*std::max_element(ra.begin(), ra.end()) / (1. - under_pb->getDiscount()));
            }
            else
            {
                std::vector<double> ra;
                for (auto &a : under_pb->getActionSpace()->getAll())
                {
                    ra.push_back(std::numeric_limits<double>::max());
                    for (auto &s : under_pb->getStateSpace()->getAll())
                    {
                        ra.back() = std::min(under_pb->getReward(s, a), ra.back());
                    }
                }
                double min_rsa = *std::max_element(ra.begin(), ra.end());
                for (int t = 0; t < vf->getHorizon(); t++)
                {
                    vf->initialize(min_rsa * (vf->getHorizon() - t), t);
                }
            }
        }
    };

    // template <typename TState, typename TAction>
    // class MDPInitializer : public Initializer<TState, TAction>
    // {
    // protected:
    //     std::string algo_name_;
    //     std::shared_ptr<DiscreteMDP> mdp_problem_;
    //     double discount_;

    // public:
    //     MDPInitializer(std::string algo_name, std::shared_ptr<DiscreteMDP> problem, double discount) : algo_name_(algo_name), problem_(problem), discount_(discount)
    //     {
    //     }

    //     void init(ValueFunction<TState, TAction> *vf)
    //     {
    //         auto algo = sdm::algo::makeMappedHSVI<number, number>(this->algo_name_, this->mdp_problem_);
    //         algo->do_solve();
    //         auto ubound = algo->getUpperBound();
    //         for (int t = 0; t < vf->getHorizon(); t++)
    //         {
    //             double max = ubound->getValueAt(this->mdp_problem_->getStateSpace()->getAll()[0], t), newval;
    //             for (auto &s : this->mdp_problem_->getStateSpace()->getAll())
    //             {
    //                 newval = ubound->getValueAt(s, t);
    //                 max = (newval > max) ? newval : max;
    //             }
    //             vf->initialize(max, t);
    //         }
    //     }
    // };
} // namespace sdm

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
#include <sdm/algorithms/hsvi.hpp>
#include <sdm/utils/value_function/value_function.hpp>
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
        virtual void init(ValueFunction<TState, TAction> *vf) = 0;
        virtual ~Initializer() {}
    };

    /**
     * @brief  This initializer initializes a value function to a constant value.
     * 
     * @tparam TState the state type
     * @tparam TAction the action type
     */
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

        void init(ValueFunction<TState, TAction> *vf)
        {
            auto under_pb = vf->getWorld()->getUnderlyingProblem();

            if (vf->isInfiniteHorizon())
            {
                // long l = log(1 - this->discount_) * this->error_ / this->reward_->getMaxReward();
                assert(under_pb->getDiscount() < 1);
                double value;
                double factor = 0, comp = 0;
                int n = 0;
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
                for (int t = vf->getHorizon() - 1; t >= 0; t--)
                {
                    if (vf->getWorld()->isSerialized())
                    {
                        if ((t + 1) % under_pb->getNumAgents() == 0)
                        {
                            tot = this->value_ + under_pb->getDiscount() * tot;
                        }
                    }
                    else
                    {
                        tot = this->value_ + under_pb->getDiscount() * tot;
                    }
                    vf->initialize(tot, t);
                }
            }
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

        void init(ValueFunction<TState, TAction> *vf)
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

        void init(ValueFunction<TState, TAction> *vf)
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
        BlindInitializer()
        {
            std::cout << "In BlindInitalizer" << std::endl;
        }

        void init(ValueFunction<TState, TAction> *vf)
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
                for (int t = vf->getHorizon() - 1; t >= 0; t--)
                {
                    if (vf->getWorld()->isSerialized())
                    {
                        if ((t + 1) % under_pb->getNumAgents() == 0)
                        {
                            tot = min_rsa + under_pb->getDiscount() * tot;
                        }
                    }
                    else
                    {
                        tot = min_rsa + under_pb->getDiscount() * tot;
                    }
                    vf->initialize(tot, t);
                }
            }
        }
    };

    template <typename TState, typename TAction>
    class PolicyEvaluationInitializer : public Initializer<TState, TAction>
    {
    public:
        PolicyEvaluationInitializer()
        {
            std::cout << "In PolicyEvaluationInitializer" << std::endl;
        }

        void init(ValueFunction<TState, TAction> *vf)
        {
            auto under_pb = vf->getWorld()->getUnderlyingProblem();

            // Je vais évaluer chaque politique en fonction d'une Value fonction correcte
            auto lb_init = sdm::BlindInitializer<TState,TAction>();
            lb_init.init(vf);

            for (auto &s : under_pb->getStateSpace()->getAll())
            {
                double resultat =std::numeric_limits<double>::min(),tot = 0;
                for (const auto &a : under_pb->getActionSpace()->getAll())
                {
                    // ra.push_back(std::numeric_limits<double>::max());
                    // for (auto &s : under_pb->getStateSpace()->getAll())
                    // {
                    //    ra.back() = std::min(policyEvaluation(a,s,vf), ra.back());
                    // }
                    resultat = std::max(policyEvaluation(a,s,vf), resultat);
                    //ra.push_back(policyEvaluation(a,s,vf));
                }
                std::cout<<"\n s : "<<s;
                std::cout<<"\n resultat : "<<resultat;
                for (int t = vf->getHorizon() - 1; t >= 0; t--)
                {
                    if (vf->getWorld()->isSerialized())
                    {
                        if ((t + 1) % under_pb->getNumAgents() == 0)
                        {
                            tot = resultat + under_pb->getDiscount() * tot;
                        }
                    }
                    else
                    {
                        tot = resultat + under_pb->getDiscount() * tot;
                    }
                    //vf->updateValueAt(s,t);
                }
            }

            // for (const auto &a : under_pb->getActionSpace()->getAll())
            // {
            //     // ra.push_back(std::numeric_limits<double>::max());
            //     // for (auto &s : under_pb->getStateSpace()->getAll())
            //     // {
            //     //    ra.back() = std::min(policyEvaluation(a,s,vf), ra.back());
            //     // }
            //     ra.push_back(policyEvaluation(a,under_pb->getInternalState(),vf));
            // }

            std::cout<<vf->str();

            // if (vf->isInfiniteHorizon())
            // {
            //     vf->initialize(*std::max_element(ra.begin(), ra.end()) / (1. - under_pb->getDiscount()));
            // }
            // else
            // {
            //     double min_rsa = *std::max_element(ra.begin(), ra.end()), tot = 0;
            //     for (int t = vf->getHorizon() - 1; t >= 0; t--)
            //     {
            //         if (vf->getWorld()->isSerialized())
            //         {
            //             if ((t + 1) % under_pb->getNumAgents() == 0)
            //             {
            //                 tot = min_rsa + under_pb->getDiscount() * tot;
            //             }
            //         }
            //         else
            //         {
            //             tot = min_rsa + under_pb->getDiscount() * tot;
            //         }
            //         vf->initialize(tot, t);
            //     }
            // }
        }

        double policyEvaluation(const number &politique, const number &state, ValueFunction<TState, TAction> *vf)
        {
            auto under_pb = vf->getWorld()->getUnderlyingProblem();
            double reward= under_pb->getReward(state, politique);
            double resultat = 0.0;
            for(auto &s_2 : under_pb->getStateSpace()->getAll())
            {
                resultat += under_pb->getStateDynamics()->getTransitionProbability(state, politique, s_2)*(reward+ under_pb->getDiscount()*vf->getValueAt(s_2));
            }
            return resultat;
        }

        double policyEvaluation(const Joint<number> &politique, const number &state, ValueFunction<TState, TAction> *vf)
        {
            auto under_pb = vf->getWorld()->getUnderlyingProblem();
            double reward= under_pb->getReward(state, politique);
            double resultat = 0.0;
            for(auto &s_2 : under_pb->getStateSpace()->getAll())
            {
                resultat += under_pb->getStateDynamics()->getTransitionProbability(state, under_pb->getActionSpace()->getJointItemIndex(politique), s_2)*(reward+ under_pb->getDiscount()*vf->getValueAt(s_2));
            }
            return resultat;
        }
    };
}// namespace sdm

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
#include <sdm/utils/value_function/value_function.hpp>


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

    // template <typename T>
    // auto createInstance() { return std::shared_ptr<T>(new T); }

    // template <typename TState, typename TAction>
    // class InitializerFactory
    // {
    // protected:
    //     typedef std::map<std::string, std::shared_ptr<Initializer<TState, TAction>> (*)()> map_type;
    //     static map_type registry;

    // public:
    //     InitializerFactory()
    //     {
    //         this->registry["MaxInitializer"] = &createInstance<MaxInitializer<TState, TAction>>;
    //         this->registry["MinInitializer"] = &createInstance<MaxInitializer<TState, TAction>>;
    //     }

    //     map_type getRegistry()
    //     {
    //         return this->registry;
    //     }

    //     std::shared_ptr<Initializer<TState, TAction>> make(std::string name)
    //     {
    //         map_type::iterator it = getFactory()->find(name);
    //         if (it == getRegistry()->end())
    //             return 0;
    //         return it->second();
    //     }

    //     // void add(std::string, Initializer *(*)()){

    //     // }
    // };

    // template <typename TState, typename TAction>
    // std::shared_ptr<Initializer<TState, TAction>> makeInitializer(std::string init_name)
    // {
    //     return InitializerFactory<TState, TAction>::make(init_name);
    // }

    /*
    template <typename TState, typename TAction>
    class MDPInitializer : public Initializer<TState, TAction>
    {
    protected:
        std::string algo_name_;
        std::shared_ptr<DiscreteMDP> mdp_problem_;
        double discount_;

    public:
        MDPInitializer(std::string algo_name, std::shared_ptr<DiscreteMDP> problem, double discount) : algo_name_(algo_name), problem_(problem), discount_(discount)
        {
        }

        void init(ValueFunction<TState, TAction> *vf)
        {
            auto algo = sdm::algo::makeMappedHSVI<number, number>(this->algo_name_, this->mdp_problem_);
            algo->do_solve();
            auto ubound = algo->getUpperBound();
            for (int t = 0; t < vf->getHorizon(); t++)
            {
                double max = ubound->getValueAt(this->mdp_problem_->getStateSpace()->getAll()[0], t), newval;
                for (auto &s : this->mdp_problem_->getStateSpace()->getAll())
                {
                    newval = ubound->getValueAt(s, t);
                    max = (newval > max) ? newval : max;
                }
                vf->initialize(max, t);
            }
        }
    };
    */   
   
    template <typename TState, typename TAction>
    class MMDPInitializer : public Initializer<TState, TAction>
    {
    protected :
        std::string algoname;

    public:

        MMDPInitializer(){}

        void init(ValueFunction<TState, TAction> *vf)
        {
            
            auto under_pb = vf->getWorld()->getUnderlyingProblem();
            std::cout<<under_pb->getDiscount();

            algoname = "../data/world/dpomdp/mabc.dpomdp";
            // Créer une variable exprès mais pour le moment je ne sais pas comment obtenir cette valeur


            //J'aimerai bien avoir une variable pour avoir le nom de l'algo ou directement le MMDP.
            // Mais je n'arrive pas à trouver, et quand je fais under_pb->toMMDP(); il ne veut pas car toute les classes mères n'ont pas cette fonction
            //auto serial_mdp = std::make_shared<SerializedMDP<SerializedState<number>,number>>(algoname);

            //MappedVector<SerializedState<number>,number> upperbound_result;
            //auto hsvi = sdm::algo::makeMappedHSVI<SerializedState<number>, number>(serial_mdp,under_pb->getDiscount(),0,under_pb->getPlanningHorizon(),1000);


            //for (auto &s : serial_mdp->getUnderlyingProblem()->getStateSpace()->getAll())
            //{
            //    serial_mdp->getUnderlyingProblem()->setInternalState(s);
                //auto hsvi = sdm::algo::makeMappedHSVI<SerializedState<number>, number>(serial_mdp,under_pb->getDiscount(),0,under_pb->getPlanningHorizon(),1000);
                //hsvi->do_solve();
                //upperbound_result[s] = hsvi->getUpperBound();
            //}
            //std::cout<<upperbound_result;
            /*
            auto algo = sdm::algo::makeMappedHSVI<number, number>(this->algo_name_, this->mmdp_problem_);
            algo->do_solve();
            auto ubound = algo->getUpperBound();
            for (int t = 0; t < vf->getHorizon(); t++)
            {
                double max = ubound->getValueAt(this->mdp_problem_->getStateSpace()->getAll()[0], t), newval;
                for (auto &s : this->mdp_problem_->getStateSpace()->getAll())
                {
                    newval = ubound->getValueAt(s, t);
                    max = (newval > max) ? newval : max;
                }
                vf->initialize(max, t);
            }*/

            //auto lb_init = std::make_shared<sdm::MinInitializer<TState, TAction>>();
            //auto ub_init = std::make_shared<sdm::MaxInitializer<TState, TAction>>();

            // Instanciate bounds
            //std::shared_ptr<sdm::ValueFunction<TState, TAction>> upper_bound(new sdm::MappedValueFunction<TState, TAction>(serial_mdp, under_pb->getPlanningHorizon(), ub_init));
            //std::shared_ptr<sdm::ValueFunction<TState, TAction>> lower_bound(new sdm::MappedValueFunction<TState, TAction>(serial_mdp, under_pb->getPlanningHorizon(), lb_init));

            //std::cout<<lower_bound->str()<<"\n";
            //std::cout<<upper_bound->str()<<"\n";

            //std::make_shared<HSVI<TState, TAction>>(problem, lower_bound, upper_bound, horizon, error, trials, name);
        }
    };
}// namespace sdm

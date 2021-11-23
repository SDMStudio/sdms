/**
 * @file initializers.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief This file contains the initializer factory class.
 * @version 1.0
 * @date 24/03/2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

#include <map>
#include <boost/bind.hpp>
#include <boost/function.hpp>

#include <sdm/utils/struct/vector.hpp>
#include <sdm/utils/value_function/initializer/initializer.hpp>
#include <sdm/utils/value_function/initializer/mdp_initializer.hpp>
#include <sdm/utils/value_function/initializer/pomdp_initializer.hpp>

namespace sdm
{
    template <class TInit>
    std::shared_ptr<Initializer> createInstance(std::shared_ptr<SolvableByHSVI> world) { return std::make_shared<TInit>(world); }

    template <class TInit>
    std::shared_ptr<Initializer> createInstanceAlgoInit(std::shared_ptr<SolvableByHSVI> world, std::string algo_name) { return std::make_shared<TInit>(world, algo_name); }

    std::shared_ptr<Initializer> createInstanceMdpHsviInit(std::shared_ptr<SolvableByHSVI> world) { return boost::bind(createInstanceAlgoInit<MDPInitializer>, world, "HSVI")(); }

    std::shared_ptr<Initializer> createInstanceMdpValueIterationInit(std::shared_ptr<SolvableByHSVI> world) { return boost::bind(createInstanceAlgoInit<MDPInitializer>, world, "ValueIteration")(); }

    std::shared_ptr<Initializer> createInstanceTabPomdpHsviInit(std::shared_ptr<SolvableByHSVI> world) { return boost::bind(createInstanceAlgoInit<POMDPInitializer>, world, "TabHSVI")(); }

    std::shared_ptr<Initializer> createInstancePomdpHsviInit(std::shared_ptr<SolvableByHSVI> world) { return boost::bind(createInstanceAlgoInit<POMDPInitializer>, world, "HSVI")(); }

    // template <class TInitializer, class... TParams>
    // std::shared_ptr<Initializer> createInstanceWithParam(std::shared_ptr<SolvableByHSVI> world, const TParams... &params) { return std::make_shared<TInitializer>(world, params...); }

    // template <class TInitializer, const char *name>
    // std::shared_ptr<Initializer> createInstance(std::shared_ptr<SolvableByHSVI> world)
    // {
    //     return boost::bind(createInstanceWithParam<TInitializer>, world, "HSVI")();
    // }

    /**
     * @brief The InitializerFactor class facilitates users to interact and instanciate value function initializers. 
     * Some of the available initializers are : MinInitializer, MaxInitializer, BlindInitializer, ZeroInitializer. For a complete list of initializer, you can use :
     *      std::cout << InitializerFactory::available() << std::endl;
     *
     */
    class InitializerFactory
    {
    protected:
        typedef std::map<std::string, std::shared_ptr<Initializer> (*)(std::shared_ptr<SolvableByHSVI> world)> map_type;
        static inline map_type registry_ = {
            {"Min", &createInstance<MinInitializer>},
            {"Max", &createInstance<MaxInitializer>},
            {"Blind", &createInstance<BlindInitializer>},
            {"Zero", &createInstance<ZeroInitializer>},
            //{"PolicyEvaluationInitializer", &createInstance,PolicyEvaluationInitializer>},
            {"Mdp", &createInstanceMdpValueIterationInit},
            {"MdpValueIteration", &createInstanceMdpValueIterationInit},
            {"MdpHsvi", &createInstanceMdpHsviInit},
            {"Pomdp", &createInstancePomdpHsviInit},
            {"PomdpHsvi", &createInstancePomdpHsviInit},
            {"TabPomdpHsvi", &createInstanceTabPomdpHsviInit},
        };

    public:
        static map_type &getRegistry()
        {
            return registry_;
        }

        static std::vector<std::string> available()
        {
            std::vector<std::string> available_init;
            for (auto const &pair_init : registry_)
                available_init.push_back(pair_init.first);
            return available_init;
        }

        static std::shared_ptr<Initializer> make(std::string name, std::shared_ptr<SolvableByHSVI> world)
        {
            typename map_type::iterator it = InitializerFactory::getRegistry().find(name);
            if (it == InitializerFactory::getRegistry().end())
            {
                std::string init_names = "{";
                for (auto &v : InitializerFactory::available())
                {
                    init_names = init_names + "\"" + v + "\" ";
                }
                throw sdm::exception::Exception(name + " not registered. Available initializers are : " + init_names + "}");
            }
            return it->second(world);
        }

        template <class TInitializer>
        static void addToRegistry(std::string name)
        {
            if (getRegistry().find(name) == getRegistry().end())
            {
                InitializerFactory::getRegistry().insert(std::make_pair(name, &createInstance<TInitializer>));
            }
            else
            {
                throw sdm::exception::Exception("Key '" + name + "' already used in Initializer Registry.");
            }
        }
    };

    std::shared_ptr<Initializer> makeInitializer(std::string init_name, std::shared_ptr<SolvableByHSVI> world)
    {
        return InitializerFactory::make(init_name, world);
    }

} // namespace sdm

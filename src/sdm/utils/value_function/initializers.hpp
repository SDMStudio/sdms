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
#include <sdm/utils/value_function/initializer.hpp>
#include <sdm/utils/value_function/initializer/mdp_initializer.hpp>

namespace sdm
{
    template <typename TState, typename TAction, template <typename TS, typename TA> class TInit>
    std::shared_ptr<Initializer<TState, TAction>> createInstance() { return std::shared_ptr<TInit<TState, TAction>>(new TInit<TState, TAction>); }

    template <typename TState, typename TAction, template <typename TS, typename TA> class TInit>
    std::shared_ptr<Initializer<TState, TAction>> createInstanceAlgoInit(std::string algo_name, double error, int trials) { return std::shared_ptr<TInit<TState, TAction>>(new TInit<TState, TAction>(algo_name, error, trials)); }

    template <typename TState, typename TAction>
    std::shared_ptr<Initializer<TState, TAction>> createInstanceTabMDPInit() { return boost::bind(createInstanceAlgoInit<TState, TAction, MDPInitializer>, "tabular_hsvi", 0., 10000)(); }

    /**
     * @brief The InitializerFactor class facilitates users to interact and instanciate value function initializers. 
     * Some of the available initializers are : MinInitializer, MaxInitializer, BlindInitializer, ZeroInitializer. For a complete list of initializer, you can use :
     * ```cpp
     * std::cout << InitializerFactory<TState, TAction>::available() << std::endl;
     * ``` 
     * 
     * @tparam TState the state type
     * @tparam TAction the action type
     */
    template <typename TState, typename TAction>
    class InitializerFactory
    {
    protected:
        typedef std::map<std::string, std::shared_ptr<Initializer<TState, TAction>> (*)()> map_type;
        static inline map_type registry_ = {
            {"MinInitializer", &createInstance<TState, TAction, MinInitializer>},
            {"MaxInitializer", &createInstance<TState, TAction, MaxInitializer>},
            {"BlindInitializer", &createInstance<TState, TAction, BlindInitializer>},
            {"ZeroInitializer", &createInstance<TState, TAction, ZeroInitializer>},
            {"MdpHsviInitializer", &createInstanceTabMDPInit<TState, TAction>},
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

        static std::shared_ptr<Initializer<TState, TAction>> make(std::string name)
        {
            typename map_type::iterator it = InitializerFactory<TState, TAction>::getRegistry().find(name);
            if (it == InitializerFactory<TState, TAction>::getRegistry().end())
            {
                std::string init_names = "{";
                for (auto &v : InitializerFactory<TState, TAction>::available())
                {
                    init_names = init_names + "\"" + v + "\" ";
                }
                throw sdm::exception::Exception(name + " not registered. Available initializers are : " + init_names + "}");
            }
            return it->second();
        }

        template <template <typename TS, typename TA> class TInitializer>
        static void addToRegistry(std::string name)
        {
            if (getRegistry().find(name) == getRegistry().end())
            {
                InitializerFactory<TState, TAction>::getRegistry().insert(std::make_pair(name, &createInstance<TState, TAction, TInitializer>));
            }
            else
            {
                throw sdm::exception::Exception("Key '" + name + "' already used in Initializer Registry.");
            }
        }
    };

    template <typename TState, typename TAction>
    std::shared_ptr<Initializer<TState, TAction>> makeInitializer(std::string init_name)
    {
        return InitializerFactory<TState, TAction>::make(init_name);
    }

} // namespace sdm

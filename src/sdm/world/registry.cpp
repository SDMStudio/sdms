#include <sdm/world/registry.hpp>
#include <sdm/config.hpp>
#include <sdm/macros.hpp>
#include <sdm/worlds.hpp>
#include <sdm/parser/parser.hpp>
#include <sdm/tools.hpp>
#include <sstream>

SDMS_REGISTRY(formalism)
SDMS_REGISTER("MDP", SolvableByMDP)
SDMS_REGISTER("BeliefMDP", BeliefMDP)
SDMS_REGISTER("OccupancyMDP", OccupancyMDP)
SDMS_REGISTER("SerialOccupancyMDP", SerialOccupancyMDP)
SDMS_REGISTER("HierarchicalOccupancyMDP", HierarchicalOccupancyMDP)
SDMS_REGISTER("bMDP", BeliefMDP)
SDMS_REGISTER("oMDP", OccupancyMDP)
SDMS_REGISTER("soMDP", SerialOccupancyMDP)
SDMS_REGISTER("hoMDP", HierarchicalOccupancyMDP)
SDMS_END_REGISTRY()

SDMS_REGISTRY(world)
SDMS_REGISTER("SerialMMDP", SerialMMDP)
SDMS_REGISTER("SerialMPOMDP", SerialMPOMDP)
SDMS_REGISTER("HierarchicalMPOMDP", HierarchicalMPOMDP)
SDMS_END_REGISTRY()

namespace sdm
{
    namespace formalism
    {
        std::vector<std::string> registry::available()
        {
            std::vector<std::string> available_init;
            for (auto const &pair_init : container)
                available_init.push_back(pair_init.first);
            return available_init;
        }

        std::shared_ptr<SolvableByHSVI> registry::make(std::string name, Config config)
        {
            typename map_type::iterator it = registry::container.find(name);
            if (it == registry::container.end())
            {
                std::string names = "{";
                for (auto &v : registry::available())
                {
                    names = names + "\"" + v + "\" ";
                }
                throw sdm::exception::Exception(name + " not registered. Available worlds are : " + names + "}");
            }
            return it->second(nullptr, config);
        }

        std::shared_ptr<SolvableByHSVI> registry::make(std::string name, const std::shared_ptr<MPOMDPInterface> &mpomdp, Config config)
        {
            typename map_type::iterator it = registry::container.find(name);
            if (it == registry::container.end())
            {
                std::string names = "{";
                for (auto &v : registry::available())
                {
                    names = names + "\"" + v + "\" ";
                }
                throw sdm::exception::Exception(name + " not registered. Available worlds are : " + names + "}");
            }
            return it->second(mpomdp, config);
        }
    }

    namespace world
    {
        std::vector<std::string> registry::available()
        {
            std::string command = "ls -l " + config::PROBLEM_PATH + "*/*.* | cut -d/ -f8";
            std::string result = sdm::tools::exec(command.c_str()), tmp;
            std::stringstream ss(result);
            std::vector<std::string> words;

            while (getline(ss, tmp, '\n'))
            {
                words.push_back(tmp);
            }
            std::vector<std::string> available_init;
            for (auto const &pair_init : container)
                available_init.push_back(pair_init.first);

            available_init.insert(available_init.end(), words.begin(), words.end());
            return available_init;
        }

        std::shared_ptr<MDPInterface> registry::make(std::string name, Config config)
        {
            typename map_type::iterator it = registry::container.find(name);
            std::shared_ptr<MDPInterface> mdp;
            if (it == registry::container.end())
            {
                auto available_worlds = available();
                if (std::find(available_worlds.begin(), available_worlds.end(), name) != available_worlds.end())
                {
                    mdp = sdm::parser::parse_file(tools::getWorldPath(name), config);
                }
                else
                {
                    std::string names = "{";
                    for (auto &v : registry::available())
                    {
                        names = names + "\"" + v + "\" ";
                    }
                    throw sdm::exception::Exception(name + " not registered. Available worlds are : " + names + "}");
                }
            }
            else
            {
                mdp = it->second(config);
            }
            return mdp;
        }

        std::shared_ptr<MDPInterface> createFromConfig(Config config)
        {
            std::shared_ptr<MDPInterface> mpomdp;
            auto mpomdp_str = config.getOpt<std::string>("world");
            auto mpomdp_config = config.getOpt<Config>("world");
            if (mpomdp_str.has_value())
                mpomdp = sdm::world::registry::make(mpomdp_str.value(), config);
            else if (mpomdp_config.has_value())
                mpomdp = sdm::world::registry::make(mpomdp_config.value().get("name", sdm::config::DEFAULT_WORLD), mpomdp_config.value());
            return mpomdp;
        }
    }
}
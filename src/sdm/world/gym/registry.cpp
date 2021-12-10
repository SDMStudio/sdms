#include <sdm/config.hpp>
#include <sdm/macros.hpp>
#include <sdm/world/gym.hpp>
#include <sdm/world/gym/registry.hpp>
#include <sdm/parser/parser.hpp>
#include <sdm/tools.hpp>
#include <sstream>

SDMS_SUB_REGISTRY(world, gym)
SDMS_REGISTER("RobotBin", RobotBin)
SDMS_END_SUB_REGISTRY()

namespace sdm
{
    namespace world
    {
        namespace gym
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
                // std::cout << result << std::endl;
                std::vector<std::string> available_init;
                for (auto const &pair_init : container)
                    available_init.push_back(pair_init.first);

                available_init.insert(available_init.end(), words.begin(), words.end());
                return available_init;
            }

            std::shared_ptr<GymInterface> registry::make(std::string name, Config config)
            {
                typename map_type::iterator it = registry::container.find(name);
                std::shared_ptr<GymInterface> gym_env;
                if (it == registry::container.end())
                {
                    auto available_worlds = available();
                    if (std::find(available_worlds.begin(), available_worlds.end(), name) != available_worlds.end())
                    {
                        gym_env = sdm::parser::parse_file(tools::getWorldPath(name), config);
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
                    gym_env = it->second(config);
                }
                return gym_env;
            }
        }
    }
}
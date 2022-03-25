#include <sdm/types.hpp>
#include <sdm/utils/config.hpp>
#include <sdm/tools.hpp>

using namespace sdm;

int main(int argc, char **argv)
{

    std::string config_path = (argc > 1) ? argv[1] : config::CONFIG_PATH,
                ex_file_1 = config_path + "example/ex-alice&bob.toml",
                ex_file_2 = config_path + "example/alice&bob.toml";

    std::cout << "--------- Usage : Configuration ( sdm/utils/config.hpp ) ---------" << std::endl
              << std::endl;

    // -------------- Instanciate configuration --------------
    std::cout << "\n\033[1;31m1. Instanciate configurations.\033[0m" << std::endl;

    std::cout << "\n\033[1;34m1. Instanciate configuration from a map.\033[0m" << std::endl;
    Config config_from_map = {
        {"num_agents", 2},
        {"agent1", Config({
                       {"name", std::string("Alice")},
                       {"sexe", std::string("F")},
                       {"height", 1.73},
                   })},
        {"agent2", Config({
                       {"name", std::string("Bob")},
                       {"sexe", std::string("M")},
                       {"height", 1.92},
                   })},
    };

    std::cout << config_from_map << std::endl;

    std::cout << "\n\033[1;34m2. Instanciate configuration from a file.\033[0m" << std::endl;
    Config config_from_file(ex_file_1);
    std::cout << config_from_file << std::endl;

    std::cout << "\n\033[1;34m3. Instanciate configuration from a file (with dependencies to other files).\033[0m" << std::endl;
    Config config_from_file_with_deps(ex_file_2);
    std::cout << config_from_file_with_deps << std::endl;

    // -------------- Instanciate configuration --------------

    return 0;
} // END main
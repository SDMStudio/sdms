#include <sdm/types.hpp>
#include <sdm/parser/parser.hpp>
#include <sdm/utils/config.hpp>
#include <sdm/worlds.hpp>
#include <sdm/utils/value_function/update_operator.hpp>

using namespace sdm;

int main(int argc, char **argv)
{

    Config lb_config = {
        {"name", "PWLCValueFunction"},
        {"initializer", "Min"},
        {"selection", "WCSP"},
        {"freq_pruning", 10},
        {"pruning_type", MaxplanPruning::PAIRWISE},
    };

    // std::cout << lb_config.get("name") << std::endl;
    // std::cout << lb_config.get("initializer") << std::endl;
    // std::cout << lb_config.get("selection") << std::endl;
    // std::cout << lb_config.get("pruning_type", MaxplanPruning::NONE) << std::endl;
    // std::cout << lb_config.get("freq_upate", 100) << std::endl;

    std::string config_file = (argc > 1) ? argv[1] : sdm::config::CONFIG_PATH + "config/hsvi.toml";

    Config algo_config(config_file);

    Config world_config = {
        {"memory", 3},
        {"batch_size", 100},
        {"state_type", 120},
    };

    std::shared_ptr<MPOMDPInterface> mpomdp = sdm::parser::parse_file(sdm::config::PROBLEM_PATH + "dpomdp/tiger.dpomdp");
    std::shared_ptr<SolvableByDP> world;

    std::cout << sdm::update::registry::available() << std::endl;

    std::cout << sdm::world::registry::available() << std::endl;
    world = sdm::world::registry::make("SerialOccupancyMDP", std::make_shared<SerialMPOMDP>(mpomdp), world_config);

    auto opt_int = world_config.getOpt<int>("state_type");
    auto opt_str = world_config.getOpt("state_type");
    if (opt_int.has_value())
    {
        std::cout << "int : " << opt_int.value() << std::endl;
    }
    else if (opt_str.has_value())
    {
        std::cout << "str : " << opt_str.value() << std::endl;
    }

    // auto algo = sdm::algo::make(algo_config.get("name"), algo_config);

    // std::cout << test.get<Config>("contacts").get<Config>("contact1").get("age", 0) << std::endl;
    // std::cout << test.get<Config>("contacts").get<Config>("contact2").get("name", "Unknown") << std::endl;
    // std::cout << test.get<Config>("contacts").get<Config>("contact2").get("age", 0) << std::endl;

    // Config algo_config = {
    //     {"type", "HSVI"},
    //     {"horizon", 10},
    //     {"discount", 0.9},
    //     {"num_max_trials", 10000},
    // };

    // Config world_config = {
    //     {"name", "tiger.dpomdp"},
    // };

    // Config formalism_config = {
    //     {"type", "OccupancyMDP"},
    //     {"world_config", world_config},
    //     {"state_type", sdm::COMPRESSED},
    //     {"store_states", true},
    //     {"store_actions", true},
    // };

    // Config omdp_hsvi_config = {
    //     {"p_b", 0.001},
    //     {"p_o", 0.01},
    //     {"p_c", 0.1},
    //     {"algo_config", algo_config},
    //     {"formalism_config", formalism_config},
    // };

    // std::cout << omdp_hsvi_config.get<double>("p_b") << std::endl;
    // std::cout << omdp_hsvi_config.get<double>("p_o") << std::endl;
    // std::cout << omdp_hsvi_config.get<Config>("algo_config").get("name", "QLearning") << std::endl;
    // std::cout << omdp_hsvi_config.get<Config>("algo_config").get("type") << std::endl;
    // std::cout << omdp_hsvi_config.get<Config>("algo_config").get("lower_bound", "maxplan") << std::endl;

    // auto value_function = sdm::update::registry::make("TabularUpdate", );
    // sdm::update::registry::make("TabularUpdate", );
    // sdm::algo::make(algo_config.get("type"), algo_config);

    // sdm::algo::make("HSVI", algo_config);
    // sdm::algo::make(algo_config.get("type"), algo_config);

    // sdm::world::make(formalism_config.get("type"), formalism_config);
    // sdm::world::make("OccupancyMDP", formalism_config);

    return 0;
} // END main
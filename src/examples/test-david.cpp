#include <sdm/types.hpp>
#include <sdm/config.hpp>
#include <sdm/utils/config.hpp>
#include <sdm/worlds.hpp>
#include <sdm/utils/value_function.hpp>
#include <sdm/algorithms/planning/hsvi.hpp>


using namespace sdm;

int main(int argc, char **argv)
{

    // double error = 0.00001;

    // std::string config_file = (argc > 1) ? argv[1] : config::CONFIG_PATH + "config/example.toml";

    // Config hsvi_config(config_file);
    // std::cout << hsvi_config << std::endl;

    // Config config_formalism = hsvi_config.get<Config>("formalism"),
    //        config_lb = hsvi_config.get<Config>("lower_bound"),
    //        config_ub = hsvi_config.get<Config>("upper_bound");

    // auto formalism = formalism::registry::make(config_formalism.get("name"), config_formalism);
    // auto lower_bound = value::registry::make(config_lb.get("name"), formalism, config_lb);
    // auto upper_bound = value::registry::make(config_ub.get("name"), formalism, config_ub);

    // auto hsvi = std::make_shared<HSVI>(formalism, lower_bound, upper_bound, hsvi_config.get("error", 0.01));

    // hsvi->initialize();
    // hsvi->solve();

    

    return 0;
} // END main
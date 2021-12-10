#include <sdm/types.hpp>
#include <sdm/parser/parser.hpp>
#include <sdm/utils/config.hpp>
#include <sdm/worlds.hpp>
#include <sdm/world/gym.hpp>
#include <sdm/utils/value_function/update_operator.hpp>

#include <sdm/world/solvable_by_hsvi.hpp>
#include <sdm/world/occupancy_mdp.hpp>
#include <sdm/algorithms/planning/hsvi.hpp>
#include <sdm/utils/value_function/action_selection/exhaustive_action_selection.hpp>
#include <sdm/utils/value_function/update_operator/vupdate/tabular_update.hpp>
#include <sdm/utils/value_function/vfunction/tabular_value_function.hpp>

using namespace sdm;

int main(int argc, char **argv)
{

	double error = 0.00001;

    std::string config_file = (argc > 1) ? argv[1] : sdm::config::CONFIG_PATH + "config/hsvi.toml";

    Config formalism_config(config_file);
    // Config gym_config(config_file);

    auto formalism = sdm::formalism::registry::make(formalism_config.get("name"), formalism_config);

    // std::cout << sdm::update::registry::available() << std::endl;
    // std::cout << sdm::formalism::registry::available() << std::endl;
    // std::cout << sdm::world::registry::available() << std::endl;
    // std::cout << sdm::world::gym::registry::available() << std::endl;

    // Instanciate Initializer
    // auto lb_init = sdm::makeInitializer("MinInitializer", formalism);
    auto lb_init = std::make_shared<MinInitializer>(formalism);
    auto ub_init = std::make_shared<MaxInitializer>(formalism);

    // Instanciate action selection
    auto action_tabular = std::make_shared<ExhaustiveActionSelection>(formalism);

    // Declare bounds
    std::shared_ptr<ValueFunction> lb, ub;

    // Instanciate lower bound
    lb = std::make_shared<TabularValueFunction>(formalism, lb_init, action_tabular);

    // Instanciate lower bound update operator
    // auto lb_update_operator = sdm::update::registry::make(lb_config.get("name"), lb);
    auto lb_update_operator = std::make_shared<TabularUpdate>(lb);
    lb->setUpdateOperator(lb_update_operator);

    // Instanciate upper bound
    ub = std::make_shared<TabularValueFunction>(formalism, ub_init, action_tabular);

    // Instanciate upper bound update operator
    // auto lb_update_operator = sdm::update::registry::make(ub_config.get("name"), ub);
    auto ub_update_operator = std::make_shared<TabularUpdate>(ub);
    ub->setUpdateOperator(ub_update_operator);

    // Instanciate HSVI
    auto algo = std::make_shared<HSVI>(formalism, lb, ub, error, 10000, "", 1, 1);

    // Initialize and solve the problem
    algo->initialize();
    algo->solve();

    return 0;
} // END main
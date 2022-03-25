#include <sdm/utils/struct/vector.hpp>
#include <sdm/types.hpp>
#include <sdm/config.hpp>
#include <sdm/worlds.hpp>
#include <sdm/world/gym.hpp>
#include <sdm/utils/value_function.hpp>
#include <sdm/utils/value_function/update_operator.hpp>
#include <sdm/utils/value_function/action_selection.hpp>

int main(int argc, char **argv)
{

    std::cout << "--------- Usage : registries ---------" << std::endl
              << std::endl;

    // -------------- GET AVAILABLE ELEMENTS IN EACH REGISTRY --------------
    std::cout << "\n\033[1;31m1. Get available classes : sdm::[NAMESPACE]::registry::available()\033[0m" << std::endl;

    std::cout << "\n\033[1;34m" << "List of FORMALISMS ( sdm/world/gym/registry.hpp )\033[0m" << std::endl;
    std::cout << sdm::formalism::registry::available() << std::endl;

    std::cout << "\n\033[1;34m" << "List of VALUE FUNCTIONS ( sdm/utils/value_function/registry.hpp )\033[0m" << std::endl;
    std::cout << sdm::value::registry::available() << std::endl;

    std::cout << "\n\033[1;34m" << "List of ACTION SELECTION ( sdm/utils/value_function/action_selection/registry.hpp )\033[0m" << std::endl;
    std::cout << sdm::action_selection::registry::available() << std::endl;

    std::cout << "\n\033[1;34m" << "List of UPDATE OPERATORS ( sdm/utils/value_function/update_operator/registry.hpp )\033[0m" << std::endl;
    std::cout << sdm::update::registry::available() << std::endl;

    std::cout << "\n\033[1;34m" << "List of WORLDS ( sdm/world/registry.hpp )\033[0m" << std::endl;
    std::cout << sdm::world::registry::available() << std::endl;

    std::cout << "\n\033[1;34m" << "List of GYM ENVIRONMENTS ( sdm/world/gym/registry.hpp )\033[0m" << std::endl;
    std::cout << sdm::world::gym::registry::available() << std::endl;


    // -------------- INSTANCIATE A CLASS FROM USING THE REGISTRY --------------
    std::cout << "\n\033[1;31m2. Instanciate a class by her name : sdm::[NAMESPACE]::registry::make(\"ClassName\", [params ,] config)\033[0m" << std::endl;

    std::cout << "\n\033[1;34m" << "Instanciate GYM ENVIRONMENT\033[0m" << std::endl;
    std::shared_ptr<sdm::GymInterface> gym_env = sdm::world::gym::registry::make("RobotBin", {{"size_x", 10}, {"size_y", 6}});

    std::cout << "\n\033[1;34m" << "Instanciate MDP\033[0m" << std::endl;
    std::shared_ptr<sdm::MDPInterface> world = sdm::world::registry::make("tiger.dpomdp", {{"horizon", 10}, {"discount", 0.99}});

    std::cout << "\n\033[1;34m" << "Instanciate Occupancy MDP\033[0m" << std::endl;
    std::shared_ptr<sdm::SolvableByHSVI> formalism = sdm::formalism::registry::make("oMDP", std::dynamic_pointer_cast<sdm::MPOMDPInterface>(world));

    sdm::Config config = {
        {"state_type", "COMPRESSED"},
        {"memory", 3},
    };
    std::cout << "\n\033[1;34m" << "Instanciate Occupancy MDP with custom config\033[0m" << std::endl;
    std::shared_ptr<sdm::SolvableByHSVI> formalism_with_config = sdm::formalism::registry::make("oMDP", std::dynamic_pointer_cast<sdm::MPOMDPInterface>(world), config);

} // END main
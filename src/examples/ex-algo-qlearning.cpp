#include <iostream>

#include <boost/program_options.hpp>

#include <sdm/types.hpp>
#include <sdm/common.hpp>
#include <sdm/algorithms/q_learning.hpp>
#include <sdm/utils/rl.hpp>
#include <sdm/utils/value_function/initializer/initializer.hpp>
#include <sdm/utils/value_function/tabular_qvalue_function.hpp>
#include <sdm/utils/value_function/backup/tabular_qvalue_backup.hpp>
#include <sdm/world/gym_interface.hpp>
#include <sdm/world/gym/RobotBin.hpp>

using namespace sdm;
using namespace std;
namespace po = boost::program_options;
int main(int argc, char **argv)
{
    int sizeX = (argc > 1) ? std::stoi(argv[1]) : 3;
    int sizeY = (argc > 2) ? std::stoi(argv[2]) : sizeX;
    number horizon = 0, num_episodes = (argc > 3) ? std::stoi(argv[3]) : 100000;
    double lr = 0.1, discount = 0.99;

    // Parse file into MPOMDP
    std::shared_ptr<GymInterface> env = std::make_shared<gym::RobotBin>(sizeX, sizeY);

    std::shared_ptr<ZeroInitializer> initializer = std::make_shared<sdm::ZeroInitializer>();
    std::shared_ptr<QValueFunction> q_value_table = std::make_shared<TabularQValueFunction>(horizon, lr, initializer);

    std::shared_ptr<ZeroInitializer> target_initializer = std::make_shared<sdm::ZeroInitializer>();
    std::shared_ptr<QValueFunction> target_q_value_table = std::make_shared<TabularQValueFunction>(horizon, lr, initializer);

    // Instanciate exploration process
    std::shared_ptr<EpsGreedy> exploration = std::make_shared<EpsGreedy>();

    // Instanciate the memory
    std::shared_ptr<ExperienceMemory> experience_memory = std::make_shared<ExperienceMemory>(horizon);

    std::shared_ptr<QValueBackupInterface> backup = std::make_shared<TabularQValueBackup>(experience_memory, q_value_table, q_value_table, discount);

    auto algorithm = std::make_shared<QLearning>(env, experience_memory, q_value_table, q_value_table, backup, exploration, horizon, discount, lr, 1, num_episodes, "RobotBin");

    algorithm->initialize();
    algorithm->solve();

} // END main

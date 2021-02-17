#include <sdm/common.hpp>
namespace sdm
{
    namespace common
    {
        void logo()
        {
            std::cout << "\033[1m\033[34m    ▄████████ ████████▄     ▄▄▄▄███▄▄▄▄   \033[33m\033[5m   ▄████████     ███     ███    █▄  ████████▄   ▄█   ▄██████▄   \033[25m\033[0m" << std::endl;
            std::cout << "\033[1m\033[34m   ███    ███ ███   ▀███  ▄██▀▀▀███▀▀▀██▄ \033[33m\033[5m  ███    ███ ▀█████████▄ ███    ███ ███   ▀███ ███  ███    ███  \033[25m\033[0m" << std::endl;
            std::cout << "\033[1m\033[34m   ███    █▀  ███    ███  ███   ███   ███ \033[33m\033[5m  ███    █▀     ▀███▀▀██ ███    ███ ███    ███ ███▌ ███    ███  \033[25m\033[0m" << std::endl;
            std::cout << "\033[1m\033[34m   ███        ███    ███  ███   ███   ███ \033[33m\033[5m  ███            ███   ▀ ███    ███ ███    ███ ███▌ ███    ███  \033[25m\033[0m" << std::endl;
            std::cout << "\033[1m\033[34m   ██████████ ███    ███  ███   ███   ███ \033[33m\033[5m  ██████████     ███     ███    ███ ███    ███ ███▌ ███    ███  \033[25m\033[0m" << std::endl;
            std::cout << "\033[1m\033[34m          ███ ███    ███  ███   ███   ███ \033[33m\033[5m         ███     ███     ███    ███ ███    ███ ███  ███    ███  \033[25m\033[0m" << std::endl;
            std::cout << "\033[1m\033[34m    ▄█    ███ ███   ▄███  ███   ███   ███ \033[33m\033[5m   ▄█    ███     ███     ███    ███ ███   ▄███ ███  ███    ███  \033[25m\033[0m" << std::endl;
            std::cout << "\033[1m\033[34m  ▄████████▀  ████████▀    ▀█   ███   █▀  \033[33m\033[5m ▄████████▀     ▄████▀   ████████▀  ████████▀  █▀    ▀██████▀   \033[25m\033[0m" << std::endl;
        }

        std::default_random_engine &global_urng()
        {
            static std::default_random_engine u{};
            return u;
        }
        
        std::string getState(state x)
        {
            std::ostringstream oss;
            oss << "state" << x;
            return oss.str();
        }

        std::string getAgentActionState(agent i, action ui, state x)
        {
            std::ostringstream oss;
            oss << "agent" << i << "." << ui << "." << x;
            return oss.str();
        }

    } // namespace common
} // namespace sdm
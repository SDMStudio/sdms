#include <sdm/common.hpp>
namespace sdm
{
    namespace common
    {
        void logo()
        {
            std::cout << std::endl;
            std::cout << config::SDMS_THEME_1 << "  ▄████████ ████████▄     ▄▄▄▄███▄▄▄▄    ▄█" << config::SDMS_THEME_2 /* <<"\033[5m" */ << "    ▄████████     ███     ███    █▄  ████████▄   ▄█   ▄██████▄ \033[0m" << std::endl;
            std::cout << config::SDMS_THEME_1 << " ███    ███ ███   ▀███  ▄██▀▀▀███▀▀▀██▄  ██" << config::SDMS_THEME_2 /* <<"\033[5m" */ << "   ███    ███ ▀█████████▄ ███    ███ ███   ▀███ ███  ███    ███\033[0m" << std::endl;
            std::cout << config::SDMS_THEME_1 << " ███    █▀  ███    ███  ███   ███   ███  ▀ " << config::SDMS_THEME_2 /* <<"\033[5m" */ << "   ███    █▀     ▀███▀▀██ ███    ███ ███    ███ ███▌ ███    ███\033[0m" << std::endl;
            std::cout << config::SDMS_THEME_1 << " ███        ███    ███  ███   ███   ███    " << config::SDMS_THEME_2 /* <<"\033[5m" */ << "   ███            ███   ▀ ███    ███ ███    ███ ███▌ ███    ███\033[0m" << std::endl;
            std::cout << config::SDMS_THEME_1 << " ██████████ ███    ███  ███   ███   ███    " << config::SDMS_THEME_2 /* <<"\033[5m" */ << "   ██████████     ███     ███    ███ ███    ███ ███▌ ███    ███\033[0m" << std::endl;
            std::cout << config::SDMS_THEME_1 << "        ███ ███    ███  ███   ███   ███    " << config::SDMS_THEME_2 /* <<"\033[5m" */ << "          ███     ███     ███    ███ ███    ███ ███  ███    ███\033[0m" << std::endl;
            std::cout << config::SDMS_THEME_1 << "  ▄█    ███ ███   ▄███  ███   ███   ███    " << config::SDMS_THEME_2 /* <<"\033[5m" */ << "    ▄█    ███     ███     ███    ███ ███   ▄███ ███  ███    ███\033[0m" << std::endl;
            std::cout << config::SDMS_THEME_1 << "▄████████▀  ████████▀    ▀█   ███   █▀     " << config::SDMS_THEME_2 /* <<"\033[5m" */ << "  ▄████████▀     ▄████▀   ████████▀  ████████▀  █▀    ▀██████▀ \033[0m" << std::endl;
            std::cout << std::endl;
        }

        std::default_random_engine &global_urng()
        {
            static std::default_random_engine u{};
            return u;
        }

        std::string getState(number state)
        {
            std::ostringstream oss;
            oss << "state" << state;
            return oss.str();
        }

        std::string getAgentActionState(number agent_id, number action_i, number state)
        {
            std::ostringstream oss;
            oss << "agent" << agent_id << "." << action_i << "." << state;
            return oss.str();
        }

    } // namespace common
} // namespace sdm
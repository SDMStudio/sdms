#include <sdm/utils/linear_programming/ndpomdp_naming.hpp>

namespace sdm
{
    std::string NDPOMDPNaming::getTransitionName(const std::shared_ptr<State> &x, const std::shared_ptr<State> &y)
    {
        std::ostringstream oss;
        oss << "tr:" << x << ":" << y;
        return oss.str();
    }
}
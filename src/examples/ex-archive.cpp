#include <fstream>

// include headers that implement a archive in simple text format
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>

#include <boost/serialization/nvp.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/version.hpp>

#include <sdm/types.hpp>
#include <sdm/world/discrete_mdp.hpp>
#include <sdm/algorithms/hsvi.hpp>
#include <sdm/utils/value_function/initializer.hpp>
#include <sdm/utils/value_function/tabular_value_function.hpp>

using namespace sdm;

/////////////////////////////////////////////////////////////
// gps coordinate
//
// illustrates serialization for a simple type
//
class gps_position
{
private:
    friend class boost::serialization::access;
    // When the class Archive corresponds to an output archive, the
    // & operator is defined similar to <<.  Likewise, when the class Archive
    // is a type of input archive the & operator is defined similar to >>.
    template <class Archive>
    void serialize(Archive &ar, const unsigned int version)
    {
        ar &BOOST_SERIALIZATION_NVP(degrees);
        ar &BOOST_SERIALIZATION_NVP(minutes);
        ar &BOOST_SERIALIZATION_NVP(seconds);
    }
    int degrees;
    int minutes;
    float seconds;

public:
    gps_position(){};
    gps_position(int d, int m, float s) : degrees(d), minutes(m), seconds(s)
    {
    }
};

int main(int argc, char **argv)
{

    std::string filename;

    if (argc > 1)
    {
        filename = argv[1];
    }

    else
    {
        std::cerr << "Error: Require 1 input file." << std::endl;
        return 1;
    }

    try
    {
        // Construct OccupancyMDP using parser
        std::cout << "#> Parsing file \"" << filename << "\"\n";

        // using TActionDescriptor = number;
        // using TStateDescriptor = HistoryTree_p<number>;

        // using TState = OccupancyState<number, JointHistoryTree_p<number>>;
        // using TAction = JointDeterministicDecisionRule<TStateDescriptor, TActionDescriptor>;

        using TState = number;
        using TAction = number;

        number horizon = 3;
        double discount = 1.0, error = 0.1, trial = 1000;

        gps_position gps(8, 8, 1.35);

        std::shared_ptr<SolvableByHSVI<TState, TAction>> omdp_world = std::make_shared<DiscreteMDP>(filename);

        // Set params in the environment
        omdp_world->getUnderlyingProblem()->setDiscount(discount);
        omdp_world->getUnderlyingProblem()->setPlanningHorizon(horizon);

        // Instanciate initializers
        auto lb_init = std::make_shared<MinInitializer<TState, TAction>>();
        auto ub_init = std::make_shared<MaxInitializer<TState, TAction>>();

        // Instanciate the max-plan representation of the lower bound
        //auto lower_bound = std::make_shared<MaxPlanValueFunction<TState, TAction>>(omdp_world, horizon, lb_init); //
        auto lower_bound = std::make_shared<MappedValueFunction<TState, TAction>>(omdp_world, horizon, lb_init);

        // Instanciate the Tabular version for the upper bound
        auto upper_bound = std::make_shared<MappedValueFunction<TState, TAction>>(omdp_world, horizon, ub_init);

        auto algo = std::make_shared<HSVI<TState, TAction>>(omdp_world, lower_bound, upper_bound, horizon, error, trial, "");

        algo->do_initialize();
        algo->do_solve();

        std::cout << *algo->getLowerBound() << std::endl;
        const auto &to_be_saved = *std::static_pointer_cast<MappedValueFunction<TState, TAction>>(algo->getLowerBound());

        // Save class into XML file
        std::ofstream ofs("archive.xml");
        // boost::archive::text_oarchive output_archive(ofs);
        boost::archive::xml_oarchive output_archive(ofs);
        output_archive << BOOST_SERIALIZATION_NVP(to_be_saved);

        // Restore class from XML file
        auto restored_value = std::make_shared<MappedValueFunction<TState, TAction>>(omdp_world, horizon, lb_init);

        // std::ifstream ifs("archive.xml");
        // boost::archive::xml_iarchive input_archive(ifs);

        // restore the schedule from the archive
        // input_archive >> BOOST_SERIALIZATION_NVP(*restored_value);

        // std::cout << "####### RESTORED VALUE FUNCTION #######\n\n";
        // std::cout << *restored_value << std::endl;
    }
    catch (exception::Exception &e)
    {
        std::cout << "!!! Exception: " << e.what() << std::endl;
    }

    return 0;
}

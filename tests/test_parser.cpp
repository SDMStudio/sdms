// #include <vector>
// #include <cppunit/ui/text/TestRunner.h>
// #include <cppunit/TestFixture.h>
// #include <cppunit/TestAssert.h>
// #include <cppunit/extensions/HelperMacros.h>

// #include <sdm/parser/parser.hpp>
// #include <sdm/exception.hpp>

// /**
//  * \class TestParserDPOMDP
//  * \brief Test Dec-POMDP's parsing from normalised files (.dpomdp format).
//  * \author David
//  * \version 1.0
//  * \date 17 nov 2020
//  *
//  */

// using namespace CppUnit;

// class TestParserDPOMDP : public CppUnit::TestFixture
// {

//   CPPUNIT_TEST_SUITE(TestParserDPOMDP);
//   CPPUNIT_TEST(testLoadTiger);
//   CPPUNIT_TEST(testFileNotFound);
//   CPPUNIT_TEST(testParsingException);
//   CPPUNIT_TEST_SUITE_END();

// protected:

// public:
//   void setUp()
//   {
//   }

//   void tearDown()
//   {
//   }

//   void testLoadTiger()
//   {
//     sdm::dpomdp tiger = sdm::parser::parse_file("data/dpomdp/tiger.dpomdp");
//     int n_state_tiger = 2, n_agent_tiger = 2, n_action_tiger = 9;
//     CPPUNIT_ASSERT_MESSAGE("Test load tiger.dpomdp", tiger.getNumStates() == n_state_tiger && tiger.getNumAgents() == n_agent_tiger && tiger.getNumActions() == n_action_tiger);
//   }


//   void testFileNotFound()
//   {
//     CPPUNIT_ASSERT_THROW(sdm::parser::parse_file("data/dpomdp/non_existing_file.dpodmp"), sdm::exception::FileNotFoundException);
//   }

//   void testParsingException()
//   {
//     CPPUNIT_ASSERT_THROW(sdm::parser::parse_file("data/ndpomdp/5P.txt"), sdm::exception::ParsingException);
//   }
// };

// ///////////////////////////////////////////////////////////////////////////////
// //  TEST PARSER PROGRAM
// ///////////////////////////////////////////////////////////////////////////////
// int main(int argc, char **argv)
// {
//   CppUnit::TextUi::TestRunner runner;
//   runner.addTest(TestParserDPOMDP::suite());
//   runner.run();
//   return 0;
// }

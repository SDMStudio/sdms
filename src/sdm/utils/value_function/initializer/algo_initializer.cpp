
// #include <sdm/algorithms.hpp>

// namespace sdm
// {
//     AlgoInitializer::AlgoInitializer(std::shared_ptr<SolvableByHSVI> world, std::string names...) : algo_name_(algo_name), error_(error), trials_(trials), world_(world)
//     {
//     }

//     void AlgoInitializer::init(std::shared_ptr<ValueFunctionInterface> value_function)
//     {
//         auto algo = makePlanning(names...);
//         algo->initialize();
//         algo->solve();
//         value_function->setInitFunction(std::make_shared<TOutput>(algo->getValueFunction));
//     }
//     // Set the function that will be used to get interactively upper bounds
// }
// } // namespace sdm


// AlgoInitializer<MDPRelaxation>
// AlgoInitializer<POMDPRelaxation>


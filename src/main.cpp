#include <ilcplex/ilocplex.h>
ILOSTLBEGIN

#include "tools/others.hpp"
#include "instance/data.hpp"
#include "solver/model.hpp"
#include "piecewise/approximation.hpp"

int main(int argc, char *argv[]) {
    greetingMessage();
    std::string parameterFile = getParameter(argc, argv);

    /* Build data */
    Data data(parameterFile);
    data.print();

    /* Build cplex environment */
    IloEnv env;
	
    try
    {
        /* Build model */
        Model model(env, data);

        /* Run model */
        model.run();

        /* Print results */
        model.printResult();
        model.output();
    }
    catch (const IloException& e) { env.end(); std::cerr << "Exception caught: " << e << std::endl; return 1; }
    catch (...) { env.end(); std::cerr << "Unknown exception caught!" << std::endl; return 1; }
    

    /*** Finalization ***/
    env.end();

   
    Approximation equidistant_inner_approx(Piecewise::buildLogFunction(2000, 0.75, 1.0), Approximation::Direction::DIRECTION_FROM_BELOW, Approximation::Method::METHOD_EQUIDISTANT, 6);
    Approximation billonnet_inner_approx(Piecewise::buildLogFunction(2000, 0.75, 1.0), Approximation::Direction::DIRECTION_FROM_BELOW, Approximation::Method::METHOD_BILLONNET, 6);
    Approximation shortest_path_inner_approx(Piecewise::buildLogFunction(2000, 0.75, 1.0), Approximation::Direction::DIRECTION_FROM_BELOW, Approximation::Method::METHOD_SHORTEST_PATH, 6);



 
    Approximation equidistant_outer_approx(Piecewise::buildLogFunction(2000, 0.75, 1.0), Approximation::Direction::DIRECTION_FROM_ABOVE, Approximation::Method::METHOD_EQUIDISTANT, 6);
    Approximation billonnet_outer_approx(Piecewise::buildLogFunction(2000, 0.75, 1.0), Approximation::Direction::DIRECTION_FROM_ABOVE, Approximation::Method::METHOD_BILLONNET, 6);
    Approximation shortest_path_outer_approx(Piecewise::buildLogFunction(2000, 0.75, 1.0), Approximation::Direction::DIRECTION_FROM_ABOVE, Approximation::Method::METHOD_SHORTEST_PATH, 6);


    std::cout << std::endl << std::endl << "RESULTS INNER APPROX : " << std::endl << std::endl;
    equidistant_inner_approx.displayApprox();
    std::cout << std::endl;
    billonnet_inner_approx.displayApprox();
    std::cout << std::endl;
    shortest_path_inner_approx.displayApprox();
    std::cout << std::endl;

    std::cout << std::endl << std::endl << "RESULTS OUTER APPROX : " << std::endl << std::endl;
    equidistant_outer_approx.displayApprox();
    std::cout << std::endl;
    billonnet_outer_approx.displayApprox();
    std::cout << std::endl;
    shortest_path_outer_approx.displayApprox();
    std::cout << std::endl;
    
    return 0;
}

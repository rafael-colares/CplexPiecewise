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


    Approximation logarithm(Piecewise::buildLogFunction(1000, 0.75, 1.0), Approximation::Direction::DIRECTION_FROM_BELOW);
    logarithm.buildApproximation(10);
    return 0;
}

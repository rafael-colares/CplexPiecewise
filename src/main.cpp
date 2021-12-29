#include <ilcplex/ilocplex.h>
ILOSTLBEGIN

#include "tools/others.hpp"
#include "instance/data.hpp"
#include "solver/model.hpp"

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
    return 0;
}

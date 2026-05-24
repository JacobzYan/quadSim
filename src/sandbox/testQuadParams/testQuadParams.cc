#include "simulator/quadParams.h"
#include <string>
#include <vector>


int main()
{




    // Read in parameter files
    std::vector<std::string> configNames = {"constants.param", "sampleQuad.param"};
    quadParams p(configNames);

    // Space out debug messages
    std::cout << "\n\n\n";
    std::cout << "Printing Quad params with 0 indents" << std::endl;
    p.printValues();

    std::cout << "Printing Quad params prop 1 with 0 indents" << std::endl;
    p.props()[0]->printValues();
    std::cout << "\n\n\n";

    std::cout << "Printing Quad params with 1 indent" << std::endl;
    p.printValues(1);
    
    return 0;
}

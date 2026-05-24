#include "simulator/quadParams.h"
#include "simulator/sim.h"
#include <string>
#include <vector>


int main()
{
    std::cout << std::endl << "STARTING TEST" << std::endl << std::endl;
    // Read in parameter files
    std::vector<std::string> configNames = {"constants.param", "defaultQuad.param"};
    quadParams p(configNames);
    std::cout << std::endl << "PARAMATERS READ" << std::endl << std::endl;


    //Set Initial State
    
    quadState::stateVector sv;
    // sv << 0,0,1.5, 0,0,0, 1,0,0,0,0,-1,0,1,0, 0,0,0, 0,0,0,0;// Rotated start
    sv << 1,1,1.5, 0,0,0, 1,0,0,0,1,0,0,0,1, 0,0,0, 0,0,0,0;// Normal start start
    
    quadState state0(sv);

    // Add to quad object
    quad defaultQuad(p, state0);
    std::cout << std::endl << "Quad Created" << std::endl;

    // Print out quad params
    Eigen::Vector4d motorVoltages;
    std::cout << std::endl << "motorVoltages Created" << std::endl;
    // motorVoltages << .1,.2,.1,.2;
    motorVoltages << .05,.05,.05,.05;
    std::cout << std::endl << "motorVoltages values populated" << std::endl;
    defaultQuad.motorVoltages = motorVoltages;
    std::cout << std::endl << "MOTOR VOLTAGES ASSIGNED" << std::endl << std::endl;
    
    
    // DEBUG

    p.printValues(); // Print out params

    // Pause, confirm user ready for simulation to take place
    std::string foo;
    std::cout << "Press enter to begin simulation:";
    // std::cin >> foo;
    std::getline(std::cin, foo);
    
    std::cout << "Starting Simulation..." << std::endl;




    // Set up Simulation
    
    std::string csvPath = "outputData.csv";
    int nStates = quad::stateVector::Zero().size();

    // Populate csv header
    std::ofstream file(csvPath);
    file << "t,";
    for(int i=0;i<nStates;i++)
    {
        file << "stateVector_" << std::to_string(i);
        if(i < nStates)
        {
            file << ",";
        }
    }
    file << "V1,V2,V3,V4";
    file << std::endl;

    std::cout << "Header Populated" << std::endl; // DEBUG

    // Solve over time
    double totalTime = 10.0;
    double dt = .01;
    int numSubsteps = 10;
    double* currentState_;
    quad::stateVector currentStateEigen;

    double substep_dt = dt / double(numSubsteps);
    int timesteps = totalTime / dt;

    
    std::cout << "Beginning timesteps" << std::endl; // DEBUG

    // Step through timesteps
    for(int i=0;i<timesteps;i++)
    {   
        defaultQuad.solve_adaptive(dt, substep_dt, true);
        // Output Solution
        currentState_ = defaultQuad.get_sol();
        file << i*dt << ","; // Fill Time col

        // Fill states
        for(int ii=0;ii<nStates;ii++)
        {
            file << *(currentState_+ii);
            
            if(ii < nStates)
            {
                file << ",";
            }
        }
        file << *defaultQuad.motorVoltages.data();
        file << std::endl;


        // DEBUG
        std::string foo;
        std::cout << "t="<<i*dt<<" - zpos="<<*(currentState_+2)<<" - Press enter to go to next timestep:";
        // std::cin >> foo;
        std::getline(std::cin, foo);

    }
    

    std::cout << "Closing File" << std::endl; // DEBUG
    // Clean up
    file.close();

    std::cout << "Exiting" << std::endl; // DEBUG

    return 0;
}

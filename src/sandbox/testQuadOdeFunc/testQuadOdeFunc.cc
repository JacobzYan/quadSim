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
    sv << 1,1,1.5, 0,0,0, 1,0,0,0,1,0,0,0,1, 0,0,0, 55,45,50,0;// Normal start start
    
    quadState state0(sv);

    // Add to quad object
    quad defaultQuad(p, state0);
    std::cout << std::endl << "Quad Created" << std::endl;

    // Print out quad params
    Eigen::Vector4d motorVoltages;
    std::cout << std::endl << "motorVoltages Created" << std::endl;
    // motorVoltages << .1,.2,.1,.2;
    // motorVoltages << .25,.25,.25,.25;
    motorVoltages << .25142341234,.2512341234123,.2514231423,.251341234;
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
    



    std::cout << "sv pre sim: " << std::endl << sv << std::endl;


    std::cout << "Starting Simulation..." << std::endl;



    // Set up dx pointer
    double x[22], dx[22];
    double *dx_ptr = dx;    
    double* x_ptr = sv.data();

    // Run one iteration of the ode fun, print
    defaultQuad.ode_fun(x_ptr, dx_ptr);
    std::cout << "sv: " << std::endl << sv << std::endl
              << "V: " << std::endl << motorVoltages << std::endl
              << "dx: " << std::endl;
    for(int i=0;i<22;i++)
    {
        std::cout << "\t" << *(dx+i) << std::endl;
    }

    std::cout << "Changing voltages and rerunning" << std::endl;
    motorVoltages << .1, .1, .1, .1;
    defaultQuad.motorVoltages = motorVoltages; 


    // rerun
    defaultQuad.ode_fun(x_ptr, dx_ptr);
    std::cout << "sv: " << std::endl << sv << std::endl
              << "V: " << std::endl << motorVoltages << std::endl
              << "dx: " << std::endl;
    for(int i=0;i<22;i++)
    {
        std::cout << "\t" << *(dx+i) << std::endl;
    }

    return 0;



    // Set up Simulation
    if(false){
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
        }
        

        std::cout << "Closing File" << std::endl; // DEBUG
        // Clean up
        file.close();

        std::cout << "Exiting" << std::endl; // DEBUG

        return 0;
    }
}

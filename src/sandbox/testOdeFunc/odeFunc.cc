#include <ode/ode_dopri_54.h>
#include <Eigen/Dense>
#include <fstream>
#include <string>
const int nStates = 2;
using stateVector = Eigen::Matrix<double, nStates, 1>;

// Temp physics/controller params
double Kp = 10.0;
double Kd = Kp / 5;
double mass = 1.0;
double l = 1.0;
double f = 1;
double g = 9.81;
double Kmotor = 5;

// Create your own class that inherits from OdeDoPri54
class MyODESystem : public ode::OdeDoPri54
{
public:
    double controlInput;
    // Constructor - set initial conditions
    MyODESystem(stateVector* x0) : ode::OdeDoPri54(nStates)
    { 
        set_sol(x0->data());  // Set initial conditions 
    }
    
    // define evolution over time
    // x[0] -> angle
    // x[1] -> angular velocity
    // x[2] -> control input
    void ode_fun(double* x, double* dydt) override
    {
        // y and dydt are raw double arrays, not Eigen vectors
        dydt[0] = x[1];
        
    // nonFrictionTorque = - mass*g*l * np.sin(x[0]) + dist[1]
    // frictionTorque = -x[1] * f  #- np.sign(x[1]) * f 
    // inputTorque = input*Kmotor
    
    // sumTorque = nonFrictionTorque + frictionTorque + inputTorque
    // thetaDoubleDot = sumTorque / (l**2 * mass)
        dydt[1] = (-x[1]*f - mass*g*l*std::sin(x[0]) + controlInput*Kmotor) / (std::pow(l,2) * mass);
    }

    // return ending state in stateVector format
    stateVector get_sol_Eigen() {return stateVector::Map(sol_);}

    // // Do something between steps
    // void after_step(double_t) override
    // {
    //     const double* sol = get_sol();

        
    // }
};



double controlLaw(double x[nStates], double xDes[nStates])
{
    double error[nStates];
    for(int i=0;i<nStates;i++)
    {
        error[i] = xDes[i] - x[i];
    }

    // DEBUG
    std::cout << "x: [" << x[0] << ", " << x[1] << "]" << std::endl;
    std::cout << "xDes: [" << xDes[0] << ", " << xDes[1] << "]" << std::endl;
    std::cout << "error: [" << error[0] <<  ", " << error[1] << "]" << std::endl;
    std::cout << "control input: " << error[0] * Kp + error[1] * Kd << std::endl;
    std::cout << "  Prop:" << error[0]*Kp << std::endl;
    std::cout << "  Deri:" << error[1]*Kd << std::endl;
    std::cout << "Torques: " << std::endl
              << "  Gravity torque: " << - mass*g*l*std::sin(x[0]) << std::endl
              << "  friction torque: " << -x[1]*f << std::endl
              << "  input torque: " << (error[0] * Kp + error[1] * Kd)*Kmotor
              << std::endl;
    std::cout << std::endl;

    return error[0] * Kp + error[1] * Kd;
}



int main()
{
    // Create an instance of your derived class
    // Will need to add control inputs to the state vector x
    stateVector x0;
    x0 << 0, -2.0;
    std::string csvPath = "output.csv";
    std::ofstream file(csvPath);
    MyODESystem solver(&x0);
    double xDes[nStates];
    xDes[0] = 3.1415926/2;
    xDes[1] = 0.0;

    // Populate csv header
    file << "t,";
    for(int i=0;i<nStates;i++)
    {
        file << "x" << std::to_string(i);
        if(i < nStates)
        {
            file << ",";
        }
    }
    file << "control";
    file << std::endl;


    // Solve over time
    double totalTime = 10.0;
    double dt = .01;
    int numSubsteps = 10;
    double* currentState_;
    stateVector current_step_eigen;

    double substep_dt = dt / double(numSubsteps);
    int timesteps = totalTime / dt;

    // Step through timesteps
    for(int i=0;i<timesteps;i++)
    {   
        double controlInput = controlLaw(solver.get_sol(),xDes);
        solver.controlInput = controlInput; 
        solver.solve_adaptive(dt, substep_dt, true);
        // Output Solution
        currentState_ = solver.get_sol();
        file << i*dt << ",";
        for(int ii=0;ii<nStates;ii++)
        {
            file << *(currentState_+ii);
            
            if(ii < nStates)
            {
                file << ",";
            }
        }
        file << controlInput;
        file << std::endl;
    }
    


    // Clean up
    file.close();
    return 0;
}
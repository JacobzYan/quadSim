#pragma once

#include <ode/ode_dopri_54.h>

#include "quadParams.h"
#include "quadController.h"
#include "quadState.h"


// const static int nNonPropStates = 17;
// // class quadParams{}; // Forward Declaration to handle circular includes


// template<size_t nProps>
// class droneState
// {   
    

//     public:

//         // Template Datatype
//         using stateVector = Eigen::Vector<double, nProps + nNonPropStates>;
//         using propVector = Eigen::Vector<double, nProps>;

//         droneState(){}
//         droneState(const stateVector & state) : stateAsVec_(state){}
//         droneState(Eigen::Map<stateVector> & stateMap) : stateAsVec_(stateMap){}
//         droneState(
//                 const quadParams & params, 
//                 const Eigen::Vector3d & pos = Eigen::Vector3d::Zero(), 
//                 const Eigen::Matrix3d & RBI = Eigen::Matrix3d::Identity(),
//                 const Eigen::Vector<double, nProps> & propOmegaB = Eigen::Vector<double, nProps>::Zero()
//                 )
//                 {
//                     this->pos() = pos; // "this" keyword is a pointer to self
//                     this->RBI() = RBI;
//                     this->propOmegaB() = propOmegaB;

//                 }
        
//         // Apply Getters/setters asVector blocks to work with the memory in stateAsVec_ directly instead of instantiating new variables - returning as const unnessecary because the return data is immutable
//         // Getters
//         Eigen::VectorBlock<const stateVector, 3> pos() const {return stateAsVec_. template segment<3>(0);}
//         Eigen::VectorBlock<const stateVector, 3> vel() const {return stateAsVec_. template segment<3>(3);}
//         const Eigen::Map<const Eigen::Matrix3d> RBI() const {return Eigen::Map<const Eigen::Matrix3d>(stateAsVec_. template segment<9>(6).data());} // Use the same data by using Map
//         Eigen::VectorBlock<const stateVector, 3> omegaB() const {return stateAsVec_. template segment<3>(15);}
//         Eigen::VectorBlock<const stateVector, nProps> propOmegaB() const {return stateAsVec_. template segment<nProps>(18);}
//         const stateVector & stateAsVec() const {return stateAsVec_;}
//         // const Eigen::Vector3d & pos() const {return stateAsVec_.segment(0,3);}
//         // const Eigen::Vector3d & vel() const {return stateAsVec_.segment(3,3);}
//         // const Eigen::Matrix3d & RBI() const {return stateAsVec_.segment(6,9).reshaped(3,3);}
//         // const Eigen::Vector3d & omegaB() const {return stateAsVec_.segment(15,3);}
//         // const propVector & propOmegaB() const {return stateAsVec_.segment(18,nProps);}
//         // const stateVector & stateAsVec() const {return stateAsVec_;}
//         // stateVector & stateAsVecMutable() { return stateAsVec_; }

//         // Setters
//         // void pos(const Eigen::Vector3d & pos){stateAsVec_.segment(0,3)=pos;}
//         // void vel(const Eigen::Vector3d & vel){stateAsVec_.segment(3,3)=vel;}
//         // void RBI(Eigen::Matrix3d & RBI){stateAsVec_.segment(6,9)=RBI.reshaped(9,1);}
//         // void omegaB(Eigen::Vector3d & omegaB){stateAsVec_.segment(15,3)=omegaB;}
//         // void propOmegaB(propVector & propOmegaB){stateAsVec_.segment(18,nProps)=propOmegaB;}
//         // void stateAsVec(stateVector & stateAsVec){stateAsVec_}
//         Eigen::VectorBlock<stateVector, 3> pos() {return stateAsVec_. template segment<3>(0);}
//         Eigen::VectorBlock<stateVector, 3> vel() {return stateAsVec_. template segment<3>(3);}
//         Eigen::Map<Eigen::Matrix3d> RBI() {return Eigen::Map< Eigen::Matrix3d>(stateAsVec_. template segment<9>(6).data());} // Use the same data by using Map
//         Eigen::VectorBlock<stateVector, 3> omegaB() {return stateAsVec_. template segment<3>(15);}
//         Eigen::VectorBlock<stateVector, nProps> propOmegaB() {return stateAsVec_. template segment<nProps>(18);}
//         stateVector & stateAsVec() {return stateAsVec_;}


//     private:
//         // Number of non-prop states to store
        
//         // Stores the quad state as a vector with the following approach:
//         // Indices: [0,2]   = rI:         3x1 Position vector of B expressed in I [m]
//         // Indices: [3,5]   = vI:         3x1 Velocity vector of B expressed in I wrt I [m/s]
//         // Indices: [6,14]  = RBI:        3x3 Attitude matrix of B wrt I stored as [RBI(1,1), RBI(2,1)...]
//         // Indices: [15,17] = omegaB:     3x1 Angular rate vector of B expressed in B wrt I [rad/s]
//         // Indices: [18,end] = propOmegaB: Nx1 Vector of prop angular rates [rad/s]
//         stateVector stateAsVec_;

// };
// using quadState = droneState<4>;






// UNUSED FOR NOW For now implement assuming 4 props - later implementations with any number of props will be desigbated as drone instead of quad
// Implement base class with all interfaces, inheriting template class for the number of props
class baseDrone : public ode::OdeDoPri54
{
    quadParams params_; // Move gravity and air density to an enviornment file
    trajectory traj_;
    // quadController controller_;
    const size_t nProps_;  
};

// Simulates the behavior of one quad
class quad : public ode::OdeDoPri54
{

    public:
        using vectorNd = quadState::VectorNd; // Eigen Double vector with length equal to number of states
        using stateVector = quadState::stateVector; // Wraps existing allocated memory in an Eigen datatype. Init with: "quadState::VectorNd <memName>; stateVector(VectorNd.data())"
        

    private:
        quadState state_; 
        quadParams params_;
        Eigen::Vector3d distVec;
        
        // Ode Func temp variables
        Eigen::Vector4d kFVec;
        Eigen::Vector4d kNVec;
        Eigen::Vector4d omegaRDirVec;
        double FVal; // When making motor direction constant, modify this back to a 4x3 matrix
        double NVal; // When making motor direction constant, modify this back to a 4x3 matrix
        Eigen::Vector3d FVec;
        Eigen::Vector3d NVec;
        Eigen::Vector3d FgI;
        double epsilon_vI = 1e-3; // This should probably be in a setup/enviornment params
        double Fd; // drag force
        Eigen::Matrix<double,3,4> FBaseMat, FMat;

        static inline stateVector defaultStateVector();
        const quadState & defaultquadState()
        {

            
            // stateVector* memStatePtr = this.sol_&; // MAKE THIS FUNCTION MAP THE STATE VECTOR TO THE INTERNAL ODE SOLVER ARRAY FRIST THEN ASSIGN VALUES

            static const quadState defaultState{};
            
            return defaultState;
        }
        

    public:
        Eigen::Array<double, 4, 1> motorVoltages, cmVec, tauMVec;
        
        
        // Constructor
        quad(const quadParams & params):  
            quad(params, defaultquadState())
            {} // Default starting pose of all 0/identity

        quad(const quadParams & params, const quadState & state0):
        
            params_(params), 
            ode::OdeDoPri54(state_.nNonPropStates_ + 4), 
            state_(sol_)

        {
            state_=state0;
            set_sol(state_.stateAsVec().data());
            updateParams();
        }

        void updateParams()
        {
            FgI << 0, 0, - params_.m() * params_.g();
            
            for(int i=0;i<4;i++)
            {
                kFVec[i] = params_.props()[i]->kF();
                kNVec[i] = params_.props()[i]->kN();
                omegaRDirVec[i] = params_.props()[i]->omegaRDir();
                FBaseMat.col(i) = params_.props()[i]->kF() * params_.props()[i]->R().row(2).transpose(); // Premultiply the force axis by the prop kF constant, each col only needs to be multiplied by propOmega^2
                cmVec[i] = params_.props()[i]->cm();
                tauMVec[i] = params_.props()[i]->tauM();
            }

            
        }

        void ode_fun(double* x, double* dxdt) override // Look into optimizations by reducing getter calls
        {
            
            quadState dxdtState_(dxdt); // Allocating in every loop as dxdt pointer not known till function called. Investigate replacing this with remapping the pointer in quadState if causing bottleneck
            // std::cout << "Entered ode_fun" << std::endl; //DEBUG
            
            // Package x
            Eigen::Map<Eigen::Vector<double, state_.nNonPropStates_+4>> xVec(x);
            quadState state(xVec);
            
            
            // std::cout << "\tstate packaged" << std::endl; //DEBUG

            // Determine Forces, moments
            // std::cout << "\tCalculation inputs:" << std::endl
            //           << "\t\tFBaseMat:\n" << FBaseMat << std::endl // Motor axis times prop kF
            //           << "\t\t state.propOmegaB().array().square().matrix().replicate(3,1):\n" << state.propOmegaB().array().square().replicate(1,3) << std::endl // Matrix of square of prop rates
            //           << "\t\tstate.propOmegaB().array();:\n" << state.propOmegaB().transpose().array().square() << std::endl
            //           << "\t\t Mult test: " << FBaseMat.array().rowwise() * state.propOmegaB().array().square().transpose();
            //           ; // DEBUG
            // FMat = FBaseMat * state.propOmegaB().array().square().matrix().replicate(1,3);
            FMat = (FBaseMat.array().rowwise() * state.propOmegaB().array().square().transpose()).matrix();
            // std::string foo; // DEBUG
            // std::cin >> foo; // DEBUG    
            
            // std::cout << "\t\tFMat Calculated" << std::endl; //DEBUG
            NVal = kNVec.dot((state.propOmegaB().array().square() * omegaRDirVec.array()).matrix()); // Still assuming vertical motors
            // std::cout << "\t\t NVal Calculated" << std::endl; //DEBUG
            // Assumes all motors are oriented in the B z direction - later implement poseable props, update the matrix in updateParams()
            FVec = FMat.rowwise().sum(); // Sum columns to get the overall force vector wrt body
            // std::cout << "\t\tFMat Summed" << std::endl; //DEBUG
            NVec << 0,0,NVal; // still assumes motors only point in the +z
            // std::cout << "\t\tNVec Summed" << std::endl; //DEBUG
            // There should be a better way to do this with some kind of matrix operation
            for(int i=0;i<4;i++)
            {
                NVec += params_.props()[i]->location().cross(FMat.col(i));
            }

            // std::cout << "\tForces and moments calculated" << std::endl; //DEBUG

            // Calculate drag force - set threshold to avoid numerical instability at low speeds
            if(state.vel().norm() > epsilon_vI)
            {
                Fd = 0.5*params_.Cd()*params_.Ad()*params_.pAir()*abs(state.vel().dot(state.RBI().row(2))*state.vel().norm()*state.vel().norm());
            }
            else{Fd=0.0;}

            // std::cout << "\tDrag Calculated" << std::endl; //DEBUG


            //  Find derivates of state elements
            dxdtState_.pos() = state.vel();
            dxdtState_.vel() = FgI + state.RBI().transpose()*FVec + distVec - Fd*state.vel().normalized();
            dxdtState_.RBI() = -1 * state.omegaB().asSkewSymmetric() * state.RBI();
            dxdtState_.omegaB() = (NVec - state_.omegaB().asSkewSymmetric()*params_.J()*state_.omegaB());
            dxdtState_.propOmegaB() = ((motorVoltages * cmVec - state_.propOmegaB().array()) / tauMVec).matrix() ;
            
            // std::cout << "\tDerivatives calculated" << std::endl; //DEBUG
            
            

            // Copy into output
            // std::memcpy(dxdt, dxdtVec.data(), (state_.nNonPropStates_ + 4) * sizeof(double)); // Not needed after changing stateVector to work with the same memory as dxdt

            // std::cout << "\tOutput Memory Calculated" << std::endl; // DEBUG

            // DEBUG
            
            std::cout << "Voltage, Ang. Velocity, dxdt:" << std::endl;
            for(int i=0;i<4;i++)
            {
                std::cout << "\t" << motorVoltages(i) << ", " << state_.propOmegaB()(i) << ", " << dxdt[state.nNonPropStates_+i] << std::endl;
            }
            std::cout << "\tzPos: " << state_.pos()(2) << std::endl;
            
        }

};

// Simulates the behavior of some number of drones in an enviornment
class droneSim
{
    
    std::vector<std::shared_ptr<baseDrone>> drones_;

};

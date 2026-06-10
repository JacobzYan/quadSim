#pragma once

#include <ode/ode_dopri_54.h>

#include "quadParams.h"
#include "quadController.h"
#include "quadState.h"




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
        Eigen::Vector4d kFVec, kNVec, omegaRDirVec;
        Eigen::Vector3d FVec, NVec, FgI;
        Eigen::Matrix<double,3,4> FBaseMat, NBaseMat, FMat, NMat;
        double epsilon_vI = 1e-3; // This should probably be in a setup/enviornment params
        double Fd; // drag force
        
        // Constructor helpers
        static inline stateVector defaultStateVector();
        const quadState & defaultquadState()
        {
            static const quadState defaultState{};   
            return defaultState;
        }
        

    public:
        Eigen::Array<double, 4, 1> motorVoltages, cmVec, tauMVec; // Make cm and taum private variables?
        
        
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
        } // Constructor with given initial pose


        // Updates precalculated matrices used to determine net forces, torques, and motor speeds
        void updateParams()
        {
            FgI << 0, 0, - params_.m() * params_.g();
            
            for(int i=0;i<4;i++)
            {
                kFVec[i] = params_.props()[i]->kF();
                kNVec[i] = params_.props()[i]->kN();
                omegaRDirVec[i] = params_.props()[i]->omegaRDir();
                FBaseMat.col(i) = params_.props()[i]->kF() * params_.props()[i]->R().row(2).transpose(); // Premultiply the force axis by the prop kF constant, each col only needs to be multiplied by propOmega^2
                NBaseMat.col(i) = params_.props()[i]->kN() * params_.props()[i]->R().row(2).transpose()
                                + (params_.props()[i]->kF() *
                                params_.props()[i]->location().cross(params_.props()[i]->R().row(2))); // Premultiply motor axis by kN, and cross location with motor axis, premultiply by kF, each col only needs to be multipled by propOmega^2.
                
                cmVec[i] = params_.props()[i]->cm();
                tauMVec[i] = params_.props()[i]->tauM();
            }

            
        }

        void ode_fun(double* x, double* dxdt) override // Look into optimizations by reducing getter calls
        {
            // Allocating in every loop as dxdt pointer not known till function called. Investigate replacing this with remapping the pointer in quadState if causing bottleneck
            quadState dxdtState_(dxdt); 
            
            // Determine net forces and torques in the body frame
            FMat = (FBaseMat.array().rowwise() * state_.propOmegaB().array().square().transpose()).matrix();
            NMat = (NBaseMat * state_.propOmegaB().array().square().matrix().asDiagonal()).matrix();

            FVec = FMat.rowwise().sum(); // Sum columns to get the overall force vector wrt body
            NVec = NMat.rowwise().sum(); // Sum columns to get the overall moment vector wrt body
            
            // Calculate drag force - set threshold to avoid numerical instability at low speeds
            if(state_.vel().norm() > epsilon_vI)
            {
                Fd = 0.5*params_.Cd()*params_.Ad()*params_.pAir()*abs(state_.vel().dot(state_.RBI().row(2))*state_.vel().norm()*state_.vel().norm());
            }
            else{Fd=0.0;}

            //  Find derivates of state elements
            dxdtState_.pos() = state_.vel();
            dxdtState_.vel() = FgI + state_.RBI().transpose()*FVec + distVec - Fd*state_.vel().normalized();
            dxdtState_.RBI() = -1 * state_.omegaB().asSkewSymmetric() * state_.RBI();
            dxdtState_.omegaB() = (NVec - state_.omegaB().asSkewSymmetric()*params_.J()*state_.omegaB());
            dxdtState_.propOmegaB() = ((motorVoltages * cmVec - state_.propOmegaB().array()) / tauMVec).matrix() ;    
        }

};

// Simulates the behavior of some number of drones in an enviornment - unfinished
class droneSim
{
    
    std::vector<std::shared_ptr<baseDrone>> drones_;

};

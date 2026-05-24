
#pragma once
#include "quadParams.h"




const static int nNonPropStates = 18;


template<size_t nProps>
class droneState
{   
    

    public:

        // Template Datatype
        using stateVector = Eigen::Vector<double, nProps + nNonPropStates>;
        using propVector = Eigen::Vector<double, nProps>;

        const inline static int nNonPropStates_=nNonPropStates;
        const static inline stateVector defaultState = [] 
            {
                // DEBUG
                std::cout << "Populating default state" << std::endl;
                stateVector vec = stateVector::Zero();

                // DEBUG
                std::cout << "vec.segment(0,nNonPropStates): " << vec.segment(0,nNonPropStates) << std::endl
                          << "vec: ";


                vec.segment(0,nNonPropStates) << 0,0,1, 0,0,0, 1,0,0,0,1,0,0,0,1, 0,0,0;
                // DEBUG
                std::cout << "Finished populating state" << std::endl
                          <<  vec.segment(0,nNonPropStates) << std::endl ;
                return vec;
            }
            (); // Lambda function to populate default
        
        
        



        droneState(): droneState(defaultState){}
        droneState(const stateVector & state) : stateAsVec_(state){}
        droneState(Eigen::Map<stateVector> & stateMap) : stateAsVec_(stateMap){}
        droneState(stateVector* stateVecPtr) {&stateAsVec = stateVecPtr;}
        droneState(
                const quadParams & params, 
                const Eigen::Vector3d & pos = Eigen::Vector3d::Zero(), 
                const Eigen::Matrix3d & RBI = Eigen::Matrix3d::Identity(),
                const Eigen::Vector<double, nProps> & propOmegaB = Eigen::Vector<double, nProps>::Zero()
                )
                {
                    this->pos() = pos; // "this" keyword is a pointer to self
                    this->RBI() = RBI;
                    this->propOmegaB() = propOmegaB;

                }
        
        // Apply Getters/setters asVector blocks to work with the memory in stateAsVec_ directly instead of instantiating new variables - returning as const unnessecary because the return data is immutable
        // Getters
        Eigen::VectorBlock<const stateVector, 3> pos() const {return stateAsVec_. template segment<3>(0);}
        Eigen::VectorBlock<const stateVector, 3> vel() const {return stateAsVec_. template segment<3>(3);}
        const Eigen::Map<const Eigen::Matrix3d> RBI() const {return Eigen::Map<const Eigen::Matrix3d>(stateAsVec_. template segment<9>(6).data());} // Use the same data by using Map
        Eigen::VectorBlock<const stateVector, 3> omegaB() const {return stateAsVec_. template segment<3>(15);}
        Eigen::VectorBlock<const stateVector, nProps> propOmegaB() const {return stateAsVec_. template segment<nProps>(18);}
        const stateVector & stateAsVec() const {return stateAsVec_;}
        // const Eigen::Vector3d & pos() const {return stateAsVec_.segment(0,3);}
        // const Eigen::Vector3d & vel() const {return stateAsVec_.segment(3,3);}
        // const Eigen::Matrix3d & RBI() const {return stateAsVec_.segment(6,9).reshaped(3,3);}
        // const Eigen::Vector3d & omegaB() const {return stateAsVec_.segment(15,3);}
        // const propVector & propOmegaB() const {return stateAsVec_.segment(18,nProps);}
        // const stateVector & stateAsVec() const {return stateAsVec_;}
        // stateVector & stateAsVecMutable() { return stateAsVec_; }

        // Setters
        void pos(const Eigen::Vector3d & pos){stateAsVec_.segment(0,3)=pos;}
        void vel(const Eigen::Vector3d & vel){stateAsVec_.segment(3,3)=vel;}
        void RBI(Eigen::Matrix3d & RBI){stateAsVec_.segment(6,9)=RBI.reshaped(9,1);}
        void omegaB(Eigen::Vector3d & omegaB){stateAsVec_.segment(15,3)=omegaB;}
        void propOmegaB(propVector & propOmegaB){stateAsVec_.segment(18,nProps)=propOmegaB;}
        void stateAsVec(stateVector & stateAsVec){stateAsVec_;}

        // Second set of getters? Not sure why
        Eigen::VectorBlock<stateVector, 3> pos() {return stateAsVec_. template segment<3>(0);}
        Eigen::VectorBlock<stateVector, 3> vel() {return stateAsVec_. template segment<3>(3);}
        Eigen::Map<Eigen::Matrix3d> RBI() {return Eigen::Map< Eigen::Matrix3d>(stateAsVec_. template segment<9>(6).data());} // Use the same data by using Map
        Eigen::VectorBlock<stateVector, 3> omegaB() {return stateAsVec_. template segment<3>(15);}
        Eigen::VectorBlock<stateVector, nProps> propOmegaB() {return stateAsVec_. template segment<nProps>(18);}
        stateVector & stateAsVec() {return stateAsVec_;}


    private:
        // Number of non-prop states to store
        
        // Stores the quad state as a vector with the following approach:
        // Indices: [0,2]   = rI:         3x1 Position vector of B expressed in I [m]
        // Indices: [3,5]   = vI:         3x1 Velocity vector of B expressed in I wrt I [m/s]
        // Indices: [6,14]  = RBI:        3x3 Attitude matrix of B wrt I stored as [RBI(1,1), RBI(2,1)...]
        // Indices: [15,17] = omegaB:     3x1 Angular rate vector of B expressed in B wrt I [rad/s]
        // Indices: [18,end] = propOmegaB: Nx1 Vector of prop angular rates [rad/s]
        stateVector stateAsVec_;

};
using quadState = droneState<4>;
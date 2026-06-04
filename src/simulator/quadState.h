
#pragma once
#include "quadParams.h"







template<size_t nProps>
class droneState
{   
    

    public:

        // Template Datatype
        const static int nNonPropStates_ = 18; // pos(3), vel(3), RBI(9), OmegaB(3)
        const static inline int nStates = nProps + nNonPropStates_;
        // using stateVector = Eigen::Vector<double, nStates>;

        using VectorNd = Eigen::Vector<double, nStates>; // Eigen Double vector with length equal to number of states
        using stateVector = Eigen::Map<VectorNd>; // Wraps existing allocated memory in an Eigen datatype. Init with: "quadState::VectorNd <memName>; stateVector(VectorNd.data())"
        using propVector = Eigen::Vector<double, nProps>;

    private:
        stateVector stateAsVec_;
        VectorNd ownedMemory = VectorNd::Zero();

        // Constructor Helpers
        void setDefaultState() // To call in constructors that need to populate the default vector
        {
            ownedMemory.head(nNonPropStates_) << 0,0,1, 0,0,0, 1,0,0,0,1,0,0,0,1, 0,0,0; // Default state of identity rot matrix, at rest, 1m above origin

        }

    public:


        /*
        // const static inline int nNonPropStates__=nNonPropStates_;

        const static inline stateVector defaultState;
            {
                // DEBUG
                std::cout << "Populating default state" << std::endl;
                VectorNd vec = stateVector::Zero();

                // DEBUG
                std::cout << "vec.segment(0,nNonPropStates_): " << vec.segment(0,nNonPropStates_) << std::endl
                          << "vec: ";


                vec.segment(0,nNonPropStates_) << 0,0,1, 0,0,0, 1,0,0,0,1,0,0,0,1, 0,0,0;
                // DEBUG
                std::cout << "Finished populating state" << std::endl
                          <<  vec.segment(0,nNonPropStates_) << std::endl ;
                return vec;
            }
            (); // Lambda function to populate default
        */
        
        


        // Constructors using defaultState own the memory. 
        droneState(): ownedMemory(), stateAsVec_(ownedMemory.data()){setDefaultState();} // Default uses internally allocated memory with default state
        droneState(const stateVector & state) : ownedMemory(state), stateAsVec_(ownedMemory.data()){} // Takes in initial state by value
        droneState(Eigen::Map<stateVector> & stateMap) : stateAsVec_(stateMap.data()){} // Takes an externally allocated double array wrapped with Eigen::Map and use that memory
        droneState(double* state) : stateAsVec_(state){} // Takes an externally allocated double array and use that memory - NO SIZE CHECK
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
        Eigen::VectorBlock<stateVector, 3> pos() {return stateAsVec_. template segment<3>(0);}
        Eigen::VectorBlock<stateVector, 3> vel() {return stateAsVec_. template segment<3>(3);}
        Eigen::Map<Eigen::Matrix3d> RBI() {return Eigen::Map< Eigen::Matrix3d>(stateAsVec_. template segment<9>(6).data());} // Use the same data by using Map
        Eigen::VectorBlock<stateVector, 3> omegaB() {return stateAsVec_. template segment<3>(15);}
        Eigen::VectorBlock<stateVector, nProps> propOmegaB() {return stateAsVec_. template segment<nProps>(18);}
        stateVector & stateAsVec() {return stateAsVec_;}


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
        void stateAsVec(stateVector & stateAsVec){stateAsVec_=stateAsVec;}


        // Second set of getters? Not sure why
        


    private:
        // Number of non-prop states to store
        
        // Stores the quad state as a vector with the following approach:
        // Indices: [0,2]   = rI:         3x1 Position vector of B expressed in I [m]
        // Indices: [3,5]   = vI:         3x1 Velocity vector of B expressed in I wrt I [m/s]
        // Indices: [6,14]  = RBI:        3x3 Attitude matrix of B wrt I stored as [RBI(1,1), RBI(2,1)...]
        // Indices: [15,17] = omegaB:     3x1 Angular rate vector of B expressed in B wrt I [rad/s]
        // Indices: [18,end] = propOmegaB: Nx1 Vector of prop angular rates [rad/s]
        

};
using quadState = droneState<4>;
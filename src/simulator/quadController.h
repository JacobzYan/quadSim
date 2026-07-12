#pragma once
#include <Eigen/Dense>
#include <ode/ode_dopri_54.h>

#include "quadParams.h"
#include "quadState.h"


// Passing Datastructures
struct trajectoryControllerPacket
{
    Eigen::Vector3d zBI;
    double FDes;
    trajectoryControllerPacket(const Eigen::Vector3d & zBIInput, const double FDesInput){zBI=zBIInput;FDes=FDesInput;}
    trajectoryControllerPacket(){}
};
struct poseEstimate
{
    Eigen::Vector3d position;
    Eigen::Matrix3d RBI;
};
struct controllerDemands
{
    double F; // Total demanded force
    Eigen::Vector3d NB; // Demanded torque vector expressed in the body frame
};

const static int nStates_ = 15, nDynSensor=6;
// struct stateVector // Should probably redo this to just be the vector
// {
//     Eigen::Vector<double, nStates_> stateVector;

// };


struct trajectory
{
    double dt;
    std::vector<Eigen::Vector3d> position;
    std::vector<double> heading;
};


//Controller Submodules
// Controller templates
class trajectoryControllerTemplate
{
    private:
        // Controller variables - pointers to allow easy modification of change in gains to flow down
        double m_;
        double g_;
        std::shared_ptr<quadParams> paramsPtr_;

        // Helper Variables
        Eigen::Vector3d gVector;
        Eigen::Vector3d FDesVec;
        trajectoryControllerPacket response_;
        

    public:
        // Explicitly set kp and kd
        trajectoryControllerTemplate(
                        const double m,
                        const double g=9.81
                        )
                        : g_(g), m_(m) {gVector << 0,0,m*g;}

        // Getters

        // Setters
        void m(const double m) {m_=m; gVector[2]=m_*g_;}
        void g(const double g) {g_=g; gVector[2]=g_*g_;}
        void mg(const double m, const double g) {m_=m; g_=g; gVector[2]=m_*g_;}

    // Calculate control law response
    const trajectoryControllerPacket response(
                                    const Eigen::Vector3d & x,
                                    const Eigen::Vector3d & xDot, 
                                    const Eigen::Vector3d & xDes, 
                                    const Eigen::Matrix3d & RBI,
                                    const Eigen::Vector3d & xDotDes = Eigen::Vector3d::Zero(),                                       
                                    const Eigen::Vector3d & xDotDotDes = Eigen::Vector3d::Zero()
                                    );
};
class attitudeControllerTemplate
{
        private:
        // Controller variables - pointers to allow easy modification of change in gains to flow down
        double m_;
        double g_;
        std::shared_ptr<quadParams> paramsPtr_;

        // Helper Variables
        Eigen::Vector3d FIDes;
        Eigen::Vector3d FFGravity;
        Eigen::Vector3d response_ = Eigen::Vector3d::Zero();

    public:
    // Explicitly set kp and kd
        attitudeControllerTemplate(
                        const double m,
                        const double g=9.81
                        )
                        : g_(g), m_(m) {}

        // Calculate control law response
        const Eigen::Vector3d & response(
                                        const Eigen::Matrix3d & RBI,
                                        const Eigen::Vector3d & xDes, 
                                        const Eigen::Vector3d & zDes, 
                                        const Eigen::Vector3d & omegaB,
                                        const Eigen::Matrix3d & JB
                                        );
};
class stateEstimatorTemplate
{
    private:
        
        // Reference Info storage
        std::shared_ptr<quadParams> paramsPtr_;
        std::shared_ptr<enviornment> env_Ptr;
        
        // Output
        quadState::VectorNd estStateMemory = quadState::VectorNd::Zero(); // Need to allocate memory for the map
        quadState::stateVector estState_; 

    public:
        
        // Constructors
        stateEstimatorTemplate(std::shared_ptr<quadParams> paramsPtr): paramsPtr_(paramsPtr), estState_(estStateMemory.data())
        {
            
        }

        // Estimate the state given the sensors and their respective readings - possibly make this just pull from quadParams?
        quadState::stateVector estState (const std::vector<sensorTemplate*> measSensorPointers, std::vector<std::vector<double>> sensorReadings);
};
class quadControllerTemplate
{
    private:
        // stateEstimator estimator_;
        std::shared_ptr<trajectoryControllerTemplate> trajCon_;
        std::shared_ptr<attitudeControllerTemplate> attCon_;
        std::vector<std::shared_ptr<sensorTemplate>> sensors_;
        std::shared_ptr<quadParams> paramsPtr_;
        public:
        quadControllerTemplate
        (
            std::shared_ptr<trajectoryControllerTemplate> trajCon, 
            std::shared_ptr<attitudeControllerTemplate> attCon
        ):
            trajCon_(trajCon), 
            attCon_(attCon){}

        // 
        controllerDemands getDemands(quadState state, trajectory traj);
        Eigen::VectorXd getVoltages(quadState state, trajectory traj);
};

// Trajectory Controllers
class PDTrajectoryController : public trajectoryControllerTemplate
{
    private:
        // Controller variables - pointers to allow easy modification of change in gains to flow down
        std::shared_ptr<double> kpPtr_;
        std::shared_ptr<double> kdPtr_;
        double m_;
        double g_;
        std::shared_ptr<quadParams> paramsPtr_;
        // Helper Variables
        Eigen::Vector3d gVector;
        Eigen::Vector3d FDesVec;
        Eigen::Vector3d normalizeFDesZ;
        trajectoryControllerPacket response_;
        

    public:
        // Explicitly set kp and kd
        PDTrajectoryController(
                        std::shared_ptr<double> kpPtr, 
                        std::shared_ptr<double> kdPtr,
                        const double m,
                        const double g=9.81
                        
                        )
                        : trajectoryControllerTemplate(m, g), kpPtr_(kpPtr), kdPtr_(kdPtr), g_(g), m_(m) {gVector << 0,0,m*g; normalizeFDesZ << 0,0,1;}

        // Getters
        const double kp() const {return * kpPtr_ ;}
        const double kd() const {return * kdPtr_ ;}

        // Setters
        void kp(const double kp) {* kpPtr_ = kp;}
        void kd(const double kd) {* kdPtr_ = kd;}
        void m(const double m) {m_=m; gVector[2]=m_*g_;}
        void g(const double g) {g_=g; gVector[2]=g_*g_;}
        void mg(const double m, const double g) {m_=m; g_=g; gVector[2]=m_*g_;}

    // Calculate control law response
    const trajectoryControllerPacket response(
                                    const Eigen::Vector3d & x,
                                    const Eigen::Vector3d & xDot, 
                                    const Eigen::Vector3d & xDes, 
                                    const Eigen::Matrix3d & RBI,
                                    const Eigen::Vector3d & xDotDes = Eigen::Vector3d::Zero(),                                       
                                    const Eigen::Vector3d & xDotDotDes = Eigen::Vector3d::Zero()
                                    );
};

// Attitude Controllers
class PDAttitudeController : public attitudeControllerTemplate
{
        private:
        // Controller variables - pointers to allow easy modification of change in gains to flow down
        std::shared_ptr<Eigen::Vector3d> kpPtr_;
        std::shared_ptr<Eigen::Vector3d> kdPtr_;
        double m_;
        double g_;
        std::shared_ptr<quadParams> paramsPtr_;
        // Helper Variables
        Eigen::Vector3d FIDes;
        Eigen::Vector3d FFGravity;
        Eigen::Vector3d response_ = Eigen::Vector3d::Zero();
        Eigen::Vector3d tempYI = Eigen::Vector3d::Zero();
        Eigen::Vector3d tempXI = Eigen::Vector3d::Zero();
        Eigen::Matrix3d RDes =Eigen::Matrix3d::Zero();
        Eigen::Matrix3d RErr =Eigen::Matrix3d::Zero();
        Eigen::Vector3d AAErr = Eigen::Vector3d::Zero();


    public:
    // Explicitly set kp and kd
        PDAttitudeController(
                        std::shared_ptr<Eigen::Vector3d> kpPtr, 
                        std::shared_ptr<Eigen::Vector3d> kdPtr,
                        const double m,
                        const double g=9.81
                        )
                        : attitudeControllerTemplate(m, g), kpPtr_(kpPtr), kdPtr_(kdPtr), g_(g), m_(m) {AAErr << 0,0,0;}

        // Getters
        const Eigen::Vector3d kp() const {return * kpPtr_ ;}
        const Eigen::Vector3d kd() const {return * kdPtr_ ;}

        // Setters
        void kp(const Eigen::Vector3d kp) {* kpPtr_ = kp;}
        void kd(const Eigen::Vector3d kd) {* kdPtr_ = kd;}

        // Calculate control law response
        const Eigen::Vector3d & response(
                                        const Eigen::Matrix3d & RBI,
                                        const Eigen::Vector3d & xDes, 
                                        const Eigen::Vector3d & zDes, 
                                        const Eigen::Vector3d & omegaB,
                                        const Eigen::Matrix3d & JB
                                        );
};

// State Estimators
class ukfEstimator : public stateEstimatorTemplate
{
    private:
        // Sigma Point Spacing Parameters
        int nStates=nStates_, nProp, nCorrect, nh, nzMeas, nzDyn; //nStates = number of state variables, nProp = number of measurements that affect propogation, nCorrect = number of measurements that are used to correct the current state 
        double alpha, beta, mu, lambda;
        
        const static int nf = nStates_ + nDynSensor; // n states in the dyanmics function
        const static int nSigmaf = 2*nf+1;
        const static int nReserve = std::max(50, nSigmaf); // Max number of states to reserve, downsized with conservativeResize() in implementation

        using augDynVector = Eigen::Vector<double, nf>;
        // using augMeasVector = Eigen::Vector<double, nReserve>;

        augDynVector predAugState, WDyn;
        Eigen::VectorXd predAugMeas, WMeas;


        Eigen::Matrix<double, nStates_, nSigmaf> dynSigmaPoints;
        Eigen::MatrixXd sigmaPoints, measSigmaPoints, sigmaPointMatrixH, Q, R, P, Pxz, Pzz, S;
        int sign=-1;

        quadState::VectorNd estStateMemory;
        quadState::stateVector estState_; // Need to allocate memory for the map
        Eigen::Matrix<double, nStates_, nReserve> sigmaPointMatrixF;

    public:
        ukfEstimator(std::shared_ptr<quadParams> paramsPtr): stateEstimatorTemplate(paramsPtr), nh(0), estState_(estStateMemory.data())
        {
            
        }
        quadState::stateVector estState (const std::vector<sensorTemplate*> measSensorPointers, std::vector<std::vector<double>> sensorReadings);

        const std::vector<std::vector<double>> hMeas(const std::vector<sensorTemplate*>, const quadState::stateVector & state);
        const quadState::stateVector fDyn(const Eigen::Vector<double, 6> & IMUReading, const quadState::stateVector & state);
    
};

class naiveEstimator : stateEstimatorTemplate // WIP
{
    private:
        sensorTemplate * IMUPtr;

    public:
        naiveEstimator(std::shared_ptr<quadParams> paramsPtr);
};





class naievePDController : public quadControllerTemplate
{
    std::shared_ptr<PDTrajectoryController> trajCtrlPtr_;
    std::shared_ptr<PDAttitudeController> attCtrlPtr_; 

    double placholderTrajKp = 1;
    double placeholderTrajKd = 1;

    Eigen::Vector3d placeholderAttKp = Eigen::Vector3d::Ones();
    Eigen::Vector3d placeholderAttKd = Eigen::Vector3d::Ones();

    naievePDController(): 
    trajCtrlPtr_(std::make_shared<PDTrajectoryController>(std::make_shared<double>(placholderTrajKp), std::make_shared<double>(placeholderTrajKd), 1, 9.81)), // Placeholder mass, gravity values
    attCtrlPtr_(std::make_shared<PDAttitudeController>(std::make_shared<Eigen::Vector3d>(placeholderAttKp), std::make_shared<Eigen::Vector3d>(placeholderAttKd), 1, 9.81)),
        quadControllerTemplate(trajCtrlPtr_, attCtrlPtr_)
    {}
};


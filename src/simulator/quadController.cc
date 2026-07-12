
// Want to make a class that will just give you the next state
// Keep all the info that the quad has to make decisions inside Quad
//     Needs:
//         Internal Estimator
//         Sensor readings( as input)
//         Controller

// Make simulator the actual simulation of pose
//     Needs:
//         ODE Function as to whats going on
//         Sensor Error/noise models
//         Vector of quads to simulate
//          Storage of information on the enviornment

// Quad
//     Controller
//         Angular Controller
//         Linear Controller
//     QuadParamsFunc

// Simulator - get passed prop params
//     ODE Func

#include "quadController.h"
#include "quadParams.h"




const trajectoryControllerPacket PDTrajectoryController::response(
                                    const Eigen::Vector3d & x,
                                    const Eigen::Vector3d & xDot, 
                                    const Eigen::Vector3d & xDes, 
                                    const Eigen::Matrix3d & RBI,
                                    const Eigen::Vector3d & xDotDes,                                       
                                    const Eigen::Vector3d & xDotDotDes
                                    )
    {
        // Control gains + Feed forward desired accel, gravity, normalize to extract the portion || to zRBI
        FDesVec = (*kpPtr_*(x-xDes) + *kdPtr_*(xDot-xDotDes) - gVector + m_ * xDotDotDes);
        response_.zBI = FDesVec.normalized();
        response_.FDes = FDesVec.transpose() * RBI.transpose() * normalizeFDesZ;
        return response_;
    }





const Eigen::Vector3d & PDAttitudeController::response(
                                        const Eigen::Matrix3d & RBI,
                                        const Eigen::Vector3d & xDes, 
                                        const Eigen::Vector3d & zDes, 
                                        const Eigen::Vector3d & omegaB,
                                        const Eigen::Matrix3d & JB
                                        )
        {
            // X, Y, Z axes of desired
            RDes.row(2)= zDes;
            RDes.row(1)= zDes.cross(xDes).normalized();
            RDes.row(0)= RDes.row(1).cross(zDes);
            
            RErr = RDes * RBI.transpose();
            // Populate Axis Angle Error
            AAErr(0) = RErr(2,3) - RErr(3,2);
            AAErr(1) = RErr(1,3) - RErr(3,1);
            AAErr(2) = RErr(1,2) - RErr(2,1);

            // Proportional, Derivative, Conversion from  body to Inertial 
            response_ = kpPtr_->asDiagonal() * AAErr - kdPtr_->asDiagonal() * omegaB + omegaB.asSkewSymmetric() * JB * omegaB;
            return response_;
        }





naiveEstimator::naiveEstimator(std::shared_ptr<quadParams> paramsPtr): stateEstimatorTemplate(paramsPtr)
{
    for(int i=0;i<paramsPtr->sensors().size();i++)
            {
                if(paramsPtr->sensors()[i]->type == "IMU") // THIS IS JUST PSEUDOCODE
                {
                    IMUPtr = paramsPtr->sensors()[i]; // RESOLVE THIS TYPE ISSUE
                }
            } 
}


// Assume that the fDynamics propogation sensor is 
quadState::stateVector ukfEstimator::estState (const std::vector<sensorTemplate*> measSensorPointers, std::vector<std::vector<double>> sensorReadings)
{
    // Determine size of state variables, measurements, Propogation sensors
    // nf n states in the dyanmics function (constant, declared in header)
    nh = measSensorPointers.size() + nStates;// n states in the measurement function
    predAugMeas.conservativeResize(nh);
    WMeas.conservativeResize(nh);

    // Determine sigma points, weights for meas function
    measSigmaPoints.col(0) = estState_;
    // Eigen::LLT<Eigen::Matrix<double,nh,nh>>; // Not sure how to handle this
    for(int i=1;i<measSigmaPoints.cols();i++)
    {
        measSigmaPoints.col(i) = measSigmaPoints.col(0);
    }
    

    // Determine signa points, weights for dynamics function


    
    // Pass meas sigma points through measurement functions

    // Correct Estimated State

    // Determine next state from dynamics function



    /*
    Implement various estimation techniques - UKF, 
    UKF
    State transition function f() -> takes in last est state, control inputs and tries to predict current state
    Measurement function h() -> takes in last est state, tries to predict what measurements should be
    // On initialization, must provide:
    - Starting covariance
    - Tuning parameters

    Each step, must provide:
    - previous predicted state, covariance matrix
    - Control input
    - Process noise covariance (Q) (dynamics model uncertainty)
    - Current step measurement (zk)
    - Meas noise covariance (R) (sensor/measurement model uncertainty)

    Procedure:
    - Create 2n+1 sigma points of state(x_i): x_0 = mu, x_i = mu + sqrt(P_i(n+lambda)), x_i+n = mu - sqrt(P_i(n+lambda)) -> cover center, +/- in each "direction"
        - n = number of state variables
        - mu = last est state
        - lambda = alpha^2()n+kappa-n
            - alpha, beta, kappa are gains that you determine before hand (typ a in [10^-3,1], b=2 for gaussian, kappa=0)
        - P = covariance
        - do the square root with the cholesky decomposition (sqrt(n+lambda)*P) where P=LL' -> = sqrt(n+labmda)*L
            - Each column of the final expression gives one "direction" of uncertainty
            - Result is x_i = mu + column_i, x_i+n = mu-column_i
    - Generate a weight for each component - Each non central component has the same weights, differeing distances represent the covariance likely to be measured, weight there to preserve the covariance of this discrete set of test points
        - For center point, Wm_0 -> mean weight = lambda / (n+lambda), Wc_0 = lambda / (n+lambda) + (1-alpha^2+beta)
        - for al other points, Wm_i = Wc_i -> covariance weight = 1/(2(n+lambda))
        - This ensures that SUM(i)[Wm_i*x_i] = mu, SUM(i)[Wc_i(x_i-mu)(x_i-mu)'] = P -> exists in the state space, predictd cov for the next loop

    - Pass each sigma point through f(), (distorts the point cloud) weighted average them to get the predicted next state, take their covariance to get the predicted covariance of the f() function at this operating point
    - Pass the sigma point states into the measurement model to see what measurements you expect - get the mean and covariance of those measurements
    - Get the mean of these measurements (z)
    - S = Cov(z) = P_zz + R -> combines uncertainty from sigma point spread (covariance) + sensor noise, P_zz and R exist in the measurement space not in the state space1
        - P_zz = covariance of predicted measurements from sigma points scaled by covariance weight -> P_zz = SUM(i)[Wc_i*(z_i - z_weightedMean)*(z_i - z_weightedMean)']
            - z_weightedMean = SUM(i)[Wc_i*z_i]
        - R = sensor noise covariance
            - Combine several sensors that measure the same state by combining their variance (reciprocal of sum of reciprocals of variance)
    - Pxz = Cov(x,z) = SUM(i)[Wc_i*(x_i - x_weightedMean)*(z_i - z_weightedMean)'] -> how the state affects the meas, if a state variable correlates to a measurement, affects kalman gain K (K=Pxz*S^-1), the number of summed matrices is squal to the number of sigma points, has a shape of nx * nz
    - Get Kalman gain K = P_xz * S^-1
    Update state x_k = x_predicted + K(z_k - z_k_predicted)
    IMPORTANT NOTES: Sensors used to advance the dyanmics function (IMU) are not included in the original sigma point set, split one estimator for propogation and one for sensing?
    */
    quadState::VectorNd placeholder =quadState::VectorNd::Zero(); // Just here to let the function compile till I get around to fixing it
    return quadState::stateVector(placeholder.data());
}










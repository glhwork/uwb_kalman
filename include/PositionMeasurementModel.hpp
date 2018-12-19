#ifndef KALMAN_EXAMPLES_ROBOT1_POSITIONMEASUREMENTMODEL_HPP_
#define KALMAN_EXAMPLES_ROBOT1_POSITIONMEASUREMENTMODEL_HPP_

#include <kalman/LinearizedMeasurementModel.hpp>

namespace uwb
{
namespace uwb
{

/**
 * @brief Measurement vector measuring the robot position
 *
 * @param T Numeric scalar type
 */
template<typename T>
class PositionMeasurement : public Kalman::Vector<T, 1>
{
public:
    KALMAN_VECTOR(PositionMeasurement, T, 1)
};

/**
 * @brief Measurement model for measuring the position of the robot
 *        using two beacon-landmarks
 *
 * This is the measurement model for measuring the position of the robot.
 * The measurement is given by two landmarks in the space, whose positions are known.
 * The robot can measure the direct distance to both the landmarks, for instance
 * through visual localization techniques.
 *
 * @param T Numeric scalar type
 * @param CovarianceBase Class template to determine the covariance representation
 *                       (as covariance matrix (StandardBase) or as lower-triangular
 *                       coveriace square root (SquareRootBase))
 */
template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
class PositionMeasurementModel : public Kalman::LinearizedMeasurementModel<State<T>, PositionMeasurement<T>, CovarianceBase>
{
public:
    //! State type shortcut definition
    typedef State<T> S;
    
    //! Measurement type shortcut definition
    typedef PositionMeasurement<T> M;
    
    /**
     * @brief Constructor
     *
     * @param landmark1x The x-position of landmark 1
     * @param landmark1y The y-position of landmark 1
     * @param landmark2x The x-position of landmark 2
     * @param landmark2y The y-position of landmark 2
     */
    PositionMeasurementModel(T landmarkx, T landmarky, T landmarkz)
    {
        // Save landmark positions
        landmark << landmarkx, landmarky,landmarkz;
        this->V.setIdentity();
    }
    
    /**
     * @brief Definition of (possibly non-linear) measurement function
     *
     * This function maps the system state to the measurement that is expected
     * to be received from the sensor assuming the system is currently in the
     * estimated state.
     *
     * @param [in] x The system state in current time-step
     * @returns The (predicted) sensor measurement for the system state
     */
    M h(const S& x) const
    {
        M measurement;
        Kalman::Vector<T, 3> position = x.template head<3>();
        
        // Distance of robot to landmark 1
        Kalman::Vector<T, 3> delta = position - landmark;
        measurement[0]= std::sqrt( delta.dot(delta) );
        //std::cout << "predict measurements: " << measurement[0] << std::endl;
        return measurement;
    }
    
protected:
    Kalman::Vector<T, 3> landmark;

protected:
    
    /**
     * @brief Update jacobian matrices for the system state transition function using current state
     *
     * This will re-compute the (state-dependent) elements of the jacobian matrices
     * to linearize the non-linear measurement function \f$h(x)\f$ around the
     * current state \f$x\f$.
     *
     * @note This is only needed when implementing a LinearizedSystemModel,
     *       for usage with an ExtendedKalmanFilter or SquareRootExtendedKalmanFilter.
     *       When using a fully non-linear filter such as the UnscentedKalmanFilter
     *       or its square-root form then this is not needed.
     *
     * @param x The current system state around which to linearize
     * @param u The current system control input
     */
    void updateJacobians( const S& x )
    {
        // H = dh/dx (Jacobian of measurement function w.r.t. the state)
        this->H.setZero();

        Kalman::Vector<T, 3> position = x.template head<3>();
        
        Kalman::Vector<T, 3> delta = position - landmark;

        
        // Distances
        T d = std::sqrt( delta.dot(delta) );
        
        // partial derivative of meas.d1() w.r.t. x.x()
        this->H(0, 0) = delta[0] / d;
        this->H(0, 1) = delta[1] / d;
        this->H(0, 2) = 0.0;//delta[2] / d;
    }
};

} // namespace Robot
} // namespace KalmanExamples

#endif

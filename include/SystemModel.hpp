#ifndef KALMAN_EXAMPLES1_ROBOT_SYSTEMMODEL_HPP_
#define KALMAN_EXAMPLES1_ROBOT_SYSTEMMODEL_HPP_

#include <kalman/LinearizedSystemModel.hpp>

namespace uwb
{
namespace uwb
{

/**
 * @brief System state vector-type for a 3DOF planar robot
 *
 * This is a system state for a very simple planar robot that
 * is characterized by its (x,y)-Position and angular orientation.
 *
 * @param T Numeric scalar type
 */
template<typename T>
class State : public Kalman::Vector<T, 6>
{
public:
    KALMAN_VECTOR(State, T, 6)
};

/**
 * @brief System control-input vector-type for a 3DOF planar robot
 *
 * This is the system control-input of a very simple planar robot that
 * can control the velocity in its current direction as well as the
 * change in direction.
 *
 * @param T Numeric scalar type
 */
template<typename T>
class Control : public Kalman::Vector<T, 1>
{
public:
    KALMAN_VECTOR(Control, T, 1)
};

/**
 * @brief System model for a simple planar 3DOF robot
 *
 * This is the system model defining how our robot moves from one 
 * time-step to the next, i.e. how the system state evolves over time.
 *
 * @param T Numeric scalar type
 * @param CovarianceBase Class template to determine the covariance representation
 *                       (as covariance matrix (StandardBase) or as lower-triangular
 *                       coveriace square root (SquareRootBase))
 */
template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
class SystemModel : public Kalman::LinearizedSystemModel<State<T>, Control<T>, CovarianceBase>
{
public:
    //! State type shortcut definition
    typedef State<T> S;
    
    //! Control type shortcut definition
    typedef Control<T> C;
    
    /**
     * @brief Definition of (non-linear) state transition function
     *
     * This function defines how the system state is propagated through time,
     * i.e. it defines in which state \f$\hat{x}_{k+1}\f$ is system is expected to 
     * be in time-step \f$k+1\f$ given the current state \f$x_k\f$ in step \f$k\f$ and
     * the system control input \f$u\f$.
     *
     * @param [in] x The system state in current time-step
     * @param [in] u The control vector input
     * @returns The (predicted) system state in the next time-step
     */
    S f(const S& x, const C& u) const
    {
        //! Predicted state vector after transition
        S x_;
        double deltatime = u(0);
        x_=this->F*x;
//        std::cout << "u: " << u(0) << std::endl;
//        std::cout << "F:\n" << this->F << std::endl;
//        x_(0) = x(0) + x(3) * deltatime;
//        x_(1) = x(1) + x(4) * deltatime;
//        x_(2) = x(2) + x(5) * deltatime;
//        x_(3) = x(3);
//        x_(4) = x(4);
//        x_(5) = x(5);
        // Return transitioned state vector
        return x_;
    }
    
protected:
    /**
     * @brief Update jacobian matrices for the system state transition function using current state
     *
     * This will re-compute the (state-dependent) elements of the jacobian matrices
     * to linearize the non-linear state transition function \f$f(x,u)\f$ around the
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
    void updateJacobians( const S& x, const C& u )
    {
        // F = df/dx (Jacobian of state transition w.r.t. the state)
        this->F.setIdentity();
        //this->F.setZero();
        float deltatime=u[0];
        // partial derivative of x.x() w.r.t. x.x()
        this->F(0, 3) = deltatime;
        this->F(1, 4) = deltatime;
        this->F(2, 5) = deltatime;
        
        // W = df/dw (Jacobian of state transition w.r.t. the noise)
        this->W.setIdentity();
        // TODO: more sophisticated noise modelling
        //       i.e. The noise affects the the direction in which we move as 
        //       well as the velocity (i.e. the distance we move)
    }
};

} // namespace Robot
} // namespace KalmanExamples

#endif

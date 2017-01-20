#ifndef _POSE_ESTIMATION_BASEFILTER_CPP
#define _POSE_ESTIMATION_BASEFILTER_CPP

#include <vector>
#include <string>
#include <base/Eigen.hpp>
#include <base/Float.hpp>
#include <boost/shared_ptr.hpp>
#include <exception>
#include <pose_estimation/pose_with_velocity/PoseUKF.hpp>
#include <transformer/Transformer.hpp>
#include <pose_estimation/StreamAlignmentVerifier.hpp>
#include <base/samples/RigidBodyAcceleration.hpp>

namespace pose_estimation {

struct ProcessNoiseBase
{
        base::Matrix3d position_noise;
        base::Matrix3d orientation_noise;
        base::Matrix3d velocity_noise;
        base::Matrix3d angular_velocity_noise;

        ProcessNoiseBase() : position_noise(base::Matrix3d::Constant(base::unknown<double>())),
                         orientation_noise(base::Matrix3d::Constant(base::unknown<double>())),
                         velocity_noise(base::Matrix3d::Constant(base::unknown<double>())),
                         angular_velocity_noise(base::Matrix3d::Constant(base::unknown<double>())) {}
};

class BaseFilter {

public:
    BaseFilter();
    ~BaseFilter();
    /** Initializes the filter with a valid state and covariance (Given by the _initial_state property).
     *  Also sets the process noise (_process_noise property).
     */
    bool setupFilter(base::samples::RigidBodyState initial_state, const ProcessNoiseBase &initial_process_noise, double max_time_delta);

    /** Applies a prediction step of the filter with a given current sample time.
     * The delta time step is the difference between the last sample time and the current one.
     */
    void predictionStep(const base::Time& sample_time) throw();

    /** Return the current robot pose.
     * The seperation in this method ensures that this is done at the end of the update hook
     * of a derivated task.
     */
    base::samples::RigidBodyState getCurrentState(std::string target_frame, std::string source_frame);

    inline boost::shared_ptr<PoseUKF> getPoseEstimator(){
        return pose_estimator;
    }

private:
    boost::shared_ptr<PoseUKF> pose_estimator;
    base::Time last_sample_time;

};

} /* namespace pose_estimation */

#endif

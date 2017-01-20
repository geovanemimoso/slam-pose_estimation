#include "BaseFilter.hpp"
#include <pose_estimation/pose_with_velocity/BodyStateMeasurement.hpp>

namespace pose_estimation {

BaseFilter::BaseFilter() {
}

BaseFilter::~BaseFilter() {
}

void BaseFilter::predictionStep(const base::Time& sample_time) throw()
{
    pose_estimator->predictionStepFromSampleTime(sample_time);
}

base::samples::RigidBodyState BaseFilter::getCurrentState( std::string target_frame,std::string source_frame)
{
    // get the estimated body state
    PoseUKF::State filter_state;
    PoseUKF::Covariance filter_state_cov;
    base::Time current_sample_time = pose_estimator->getLastMeasurementTime();
    if(current_sample_time > last_sample_time && pose_estimator->getCurrentState(filter_state, filter_state_cov))
    {
        base::samples::RigidBodyState body_state;
        BodyStateMeasurement::toRigidBodyState(filter_state, filter_state_cov, body_state);
        body_state.time = current_sample_time;
        body_state.targetFrame = target_frame;
        body_state.sourceFrame = source_frame;
        last_sample_time = current_sample_time;
        return body_state;
    }
}

bool BaseFilter::setupFilter(base::samples::RigidBodyState initial_state, const ProcessNoiseBase& initial_process_noise, double max_time_delta)
{
    // setup initial state
    base::samples::RigidBodyState init_rbs(false);
    init_rbs.initUnknown();
    init_rbs.cov_position = 1.0 * base::Matrix3d::Identity();
    init_rbs.cov_orientation = 1.0 * base::Matrix3d::Identity();
    init_rbs.cov_velocity = 0.1 * base::Matrix3d::Identity();
    init_rbs.cov_angular_velocity = 0.05 * base::Matrix3d::Identity();

    base::samples::RigidBodyState override_init_state = initial_state;
    if(override_init_state.hasValidPosition())
        init_rbs.position = override_init_state.position;
    if(override_init_state.hasValidOrientation())
        init_rbs.orientation = override_init_state.orientation;
    if(override_init_state.hasValidVelocity())
        init_rbs.velocity = override_init_state.velocity;
    if(override_init_state.hasValidAngularVelocity())
        init_rbs.angular_velocity = override_init_state.angular_velocity;

    if(override_init_state.hasValidPositionCovariance())
        init_rbs.cov_position = override_init_state.cov_position;
    if(override_init_state.hasValidOrientationCovariance())
        init_rbs.cov_orientation = override_init_state.cov_orientation;
    if(override_init_state.hasValidVelocityCovariance())
        init_rbs.cov_velocity = override_init_state.cov_velocity;
    if(override_init_state.hasValidAngularVelocityCovariance())
        init_rbs.cov_angular_velocity = override_init_state.cov_angular_velocity;

    PoseUKF::State init_state;
    PoseUKF::Covariance init_state_cov;
    BodyStateMeasurement::fromRigidBodyState(init_rbs, init_state, init_state_cov);
    pose_estimator.reset(new PoseUKF(init_state, init_state_cov));

    // setup process noise
    const ProcessNoiseBase& process_noise = initial_process_noise;
    PoseUKF::Covariance process_noise_cov = PoseUKF::Covariance::Zero();

    if(base::isnotnan(process_noise.position_noise))
        process_noise_cov.block(0,0,3,3) = process_noise.position_noise;
    else
        process_noise_cov.block(0,0,3,3) = 0.01 * base::Matrix3d::Identity();
    if(base::isnotnan(process_noise.orientation_noise))
        process_noise_cov.block(3,3,3,3) = process_noise.orientation_noise;
    else
        process_noise_cov.block(3,3,3,3) = 0.001 * base::Matrix3d::Identity();
    if(base::isnotnan(process_noise.velocity_noise))
        process_noise_cov.block(6,6,3,3) = process_noise.velocity_noise;
    else
        process_noise_cov.block(6,6,3,3) = 0.001 * base::Matrix3d::Identity();
    if(base::isnotnan(process_noise.angular_velocity_noise))
        process_noise_cov.block(9,9,3,3) = process_noise.angular_velocity_noise;
    else
        process_noise_cov.block(9,9,3,3) = 0.0001 * base::Matrix3d::Identity();

    pose_estimator->setProcessNoiseCovariance(process_noise_cov);

    pose_estimator->setMaxTimeDelta(max_time_delta);

    return true;
}

} /* namespace pose_estimation */

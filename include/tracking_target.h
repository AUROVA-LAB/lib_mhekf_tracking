#ifndef _tracking_target_h_
#define _tracking_target_h_

#include <Eigen/Dense>
#include <pcl/point_types.h>

#include "observation.h"
#include "struct_definitions.h"

class TrackingTarget
{
  private:
    Eigen::Matrix <double, 5, 1> X;
    Eigen::Matrix <double, 5, 5> F_X;
    Eigen::Matrix <double, 5, 2> F_q;
    Eigen::Matrix <double, 2, 2> Q;
    Eigen::Matrix <double, 5, 5> P;
    Eigen::Matrix <double, 3, 1> y;
    Eigen::Matrix <double, 3, 1> z;
    Eigen::Matrix <double, 3, 5> H;
    Eigen::Matrix <double, 3, 3> Z;
    Eigen::Matrix <double, 3, 3> R;
    Eigen::Matrix <double, 5, 3> K;

    Eigen::Vector3d Xs, Vs;  // Pose sensor
    Eigen::Vector3d Xe, e; // expectation, innovation

    Eigen::Matrix3d Rot; // Rotation Matrix

    int iteration_number; // Used to know when it's possible to compute velocity and acceleration

    int t_secs;
    int t_nsecs;

    float initial_width;
    float initial_length;

    float width;
    float length;

    float width_to_plot;
    float length_to_plot;

    void CalculateStateJacobian(double dt);
  
    int FindClosestVertex ( float xt, float yt, float width_t, float length_t, float theta_t, 
                              pcl::PointXYZ& a, pcl::PointXYZ& b, pcl::PointXYZ& c, pcl::PointXYZ& d);

    void CalculateVertexCoordinates ( float w, float l, float x, float y, float theta,
                                        pcl::PointXYZ& a, pcl::PointXYZ& b, pcl::PointXYZ& c, pcl::PointXYZ& d );

    float CorrectingThetaUsingMahalanobisDistance(float x, float y, float theta, float ego_x, float ego_y, float ego_theta, int secs, int nsecs);

    void GetExpectedObservation ( float& x_req, float& y_req, float& theta_req, float ego_x, float ego_y, float ego_theta, int secs, int nsecs );

    //void AdaptThetaToTheCurrentQuadrant(float previous_theta, float& theta_observed);
  
  public:

    /**
    * \brief Constructor
    */
    TrackingTarget(void);

    /**
    * \brief Destructor
    */
    ~TrackingTarget(void);

    /**
    * \brief Updating coordinates
    * This function updates automatically all the parameters
    * of the class.
    */

    void SetStandardDeviations(mhekf_tracker::KalmanConfiguration kalman_configuration);

    float Update ( float ego_x, float ego_y, float ego_theta, float xt, float yt,
		   float width_t, float length_t, float theta_t, float theta_noise_factor, float mean_squared_error,
		   int secs, int nsecs, bool associated);

    float Update ( CObservation obs, float theta_noise_factor, float ego_x, float ego_y, float ego_theta, bool associated);

    void GetVariablesToPlot(float& x_req, float& y_req, float& w_req, float& l_req, float& theta_req, float& vel_req, float ego_x, float ego_y, float ego_theta);

    void GetDimensions(float& a, float& b);

    void GetCovariance ( float& cov_x, float& cov_y);

    void GetFullStateAndCovariance ( Eigen::Matrix <double, 5, 1>& state, Eigen::Matrix <double, 5, 5>& covariance);

    void CorrectingObservationByGeometry ( float& xt, float& yt, float width_t, float length_t, float& theta_t,
					   float ego_x, float ego_y, float ego_theta, int secs, int nsecs, bool store_values );

    float Likelihood(float x_obs, float y_obs, float theta_obs, float theta_noise_factor, float ego_x, float ego_y, float ego_theta, int secs, int nsecs);

    float MahalanobisDistance(float x_obs, float y_obs, float theta_obs, float theta_noise_factor,
			      float ego_x, float ego_y, float ego_theta, int secs, int nsecs);

    float EuclideanDistance(float x_obs, float y_obs, float theta_obs, float ego_x, float ego_y, float ego_theta, int secs, int nsecs);
};

#endif

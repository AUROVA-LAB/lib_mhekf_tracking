#include "tracking_target.h"
//#include "tracking_vehicles_alg.h"
#include "observation.h"
#include "struct_definitions.h"

#include "math.h"
#include <Eigen/Dense>
#include <pcl/point_types.h>

TrackingTarget::TrackingTarget(void)
{
  // Odometry vector x y theta
  Xs << 0.0, 0.0, 0.0;
  Vs << 0.0, 0.0, 0.0; // Velocity vector vx vy omega

  //State vector
  X(0, 0) = 0.0;
  X(1, 0) = 0.0;
  X(2, 0) = 0.0;
  X(3, 0) = 0.0;
  X(4, 0) = 0.0;

  // State covariance matrix
  P(0, 0) = pow(1, 2); // initial value for x covariance;
  P(0, 1) = 0.0;
  P(0, 2) = 0.0;
  P(0, 3) = 0.0;
  P(0, 4) = 0.0;

  P(1, 0) = 0.0;
  P(1, 1) = pow(1, 2); // y covariance
  P(1, 2) = 0.0;
  P(1, 3) = 0.0;
  P(1, 4) = 0.0;

  P(2, 0) = 0.0;
  P(2, 1) = 0.0;
  P(2, 2) = pow((M_PI / 8.0), 2); // orientation (theta)
  P(2, 3) = 0.0;
  P(2, 4) = 0.0;

  P(3, 0) = 0.0;
  P(3, 1) = 0.0;
  P(3, 2) = 0.0;
  P(3, 3) = pow(10.0, 2); // Aprox 100 Km / h to 0 Km / h in 5 seconds with a delta_t of 0.1 seconds
  // ( 30 / sqrt(5) ) * sqrt ( 0.1 ) --> the sqrt is because of the gaussian white noise integration

  P(3, 4) = 0.0;

  P(4, 0) = 0.0;
  P(4, 1) = 0.0;
  P(4, 2) = 0.0;
  P(4, 3) = 0.0;
  P(4, 4) = pow(0.1, 2); // Debug! --> making tests...

  // Model noise Jacobian matrix
  F_q(0, 0) = 0.0;
  F_q(0, 1) = 0.0;

  F_q(1, 0) = 0.0;
  F_q(1, 1) = 0.0;

  F_q(2, 0) = 0.0;
  F_q(2, 1) = 0.0;

  F_q(3, 0) = 1.0;
  F_q(3, 1) = 0.0;

  F_q(4, 0) = 0.0;
  F_q(4, 1) = 1.0;

  // Model noise covariance matrix
  Q(0, 0) = pow(5.0, 2.0); //(10.0, 2.0); velocity noise covariance
  Q(0, 1) = 0.0;

  Q(1, 0) = 0.0;
  Q(1, 1) = pow(0.1, 2.0); //(1.0, 2.0); k noise covariance

  // Sensor noise covarinance matrix;
  R(0, 0) = pow(1, 2.0); // We will give a value of 1 meters for the standard deviation of the detection of x and y
  R(0, 1) = 0.0;
  R(0, 2) = 0.0;

  R(1, 0) = 0.0;
  R(1, 1) = pow(1, 2.0);
  R(1, 2) = 0.0;

  R(2, 0) = 0.0;
  R(2, 1) = 0.0;
  R(2, 2) = pow((M_PI / 4), 2.0);

  iteration_number = 0; // Used to know when it's possible to compute velocity and acceleration

  t_secs = 0;
  t_nsecs = 0;

  initial_width = 0.0;
  initial_length = 0.0;

  width = 0.0;
  length = 0.0;

  width_to_plot = 0.0;
  length_to_plot = 0.0;

  CalculateStateJacobian(0.0);

  Eigen::Matrix2f sign = Eigen::Matrix2f::Identity();

  H = Eigen::Matrix <double, 3, 5>::Zero();
  Z = Eigen::Matrix <double, 3, 3>::Zero();
  R = Eigen::Matrix <double, 3, 3>::Zero();
  K = Eigen::Matrix <double, 5, 3>::Zero();
}

void TrackingTarget::SetStandardDeviations(mhekf_tracker::KalmanConfiguration kalman_configuration)
{
  // Initial values
  P(0, 0) = pow(kalman_configuration.x_ini, 2);
  P(1, 1) = pow(kalman_configuration.y_ini, 2);
  P(2, 2) = pow(kalman_configuration.theta_ini, 2);
  P(3, 3) = pow(kalman_configuration.v_ini, 2);
  P(4, 4) = pow(kalman_configuration.k_ini, 2);

  // Model noise
  Q(0, 0) = pow(kalman_configuration.v_model, 2.0);
  Q(1, 1) = pow(kalman_configuration.k_model, 2.0);

  //Sensor noise
  R(0, 0) = pow(kalman_configuration.x_sensor, 2.0);
  R(1, 1) = pow(kalman_configuration.y_sensor, 2.0);
  R(2, 2) = pow(kalman_configuration.theta_sensor, 2.0);

//  std::cout << "TrackingTarget::SetStandardDeviations --> P = " << std::endl << P << std::endl;
//  std::cout << "TrackingTarget::SetStandardDeviations --> Q = " << std::endl << Q << std::endl;
//  std::cout << "TrackingTarget::SetStandardDeviations --> R = " << std::endl << R << std::endl;
}

TrackingTarget::~TrackingTarget(void)
{

}

void TrackingTarget::CalculateVertexCoordinates(float w, float l, float x, float y, float theta, pcl::PointXYZ& a,
                                                pcl::PointXYZ& b, pcl::PointXYZ& c, pcl::PointXYZ& d)
{
  float distance_from_center_to_vertex;
  float angle_between_center_and_vertex_a, angle_between_center_and_vertex_b, angle_between_center_and_vertex_c,
        angle_between_center_and_vertex_d;

  distance_from_center_to_vertex = sqrt((l / 2.0) * (l / 2.0) + (w / 2.0) * (w / 2.0));

  angle_between_center_and_vertex_a = atan2((w / 2.0), -(l / 2.0)) + theta;
  angle_between_center_and_vertex_b = atan2((w / 2.0), (l / 2.0)) + theta;
  angle_between_center_and_vertex_c = atan2(-(w / 2.0), (l / 2.0)) + theta;
  angle_between_center_and_vertex_d = atan2(-(w / 2.0), -(l / 2.0)) + theta;

  a.x = x + distance_from_center_to_vertex * cos(angle_between_center_and_vertex_a);
  a.y = y + distance_from_center_to_vertex * sin(angle_between_center_and_vertex_a);

  b.x = x + distance_from_center_to_vertex * cos(angle_between_center_and_vertex_b);
  b.y = y + distance_from_center_to_vertex * sin(angle_between_center_and_vertex_b);

  c.x = x + distance_from_center_to_vertex * cos(angle_between_center_and_vertex_c);
  c.y = y + distance_from_center_to_vertex * sin(angle_between_center_and_vertex_c);

  d.x = x + distance_from_center_to_vertex * cos(angle_between_center_and_vertex_d);
  d.y = y + distance_from_center_to_vertex * sin(angle_between_center_and_vertex_d);

}

int TrackingTarget::FindClosestVertex(float xt, float yt, float width_t, float length_t, float theta_t,
                                      pcl::PointXYZ& a, pcl::PointXYZ& b, pcl::PointXYZ& c, pcl::PointXYZ& d)
{
  float min_distance, distance;

  int vertex = -1;

  //We find the vertex coordinates of the new observation
  CalculateVertexCoordinates(width_t, length_t, xt, yt, theta_t, a, b, c, d);

  //Now we are looking for the closest vertex

  min_distance = 1000.0; //Initializing with an impossible value (max puck range = 100.0)

  //Calculus of distance to "a"
  distance = sqrt((a.x * a.x) + (a.y * a.y));

  //If its the minimum we store zero value that means "a was the minimum"
  if (distance < min_distance)
  {
    min_distance = distance;
    vertex = 0;
  }

  //Now we do the same for the others:
  distance = sqrt((b.x * b.x) + (b.y * b.y));

  if (distance < min_distance)
  {
    min_distance = distance;
    vertex = 1;
  }

  distance = sqrt((c.x * c.x) + (c.y * c.y));

  if (distance < min_distance)
  {
    min_distance = distance;
    vertex = 2;
  }

  distance = sqrt((d.x * d.x) + (d.y * d.y));

  if (distance < min_distance)
  {
    min_distance = distance;
    vertex = 3;
  }

  return (vertex);
}

void TrackingTarget::CorrectingObservationByGeometry(float& xt, float& yt, float width_t, float length_t,
                                                     float& theta_t, float ego_x, float ego_y, float ego_theta,
                                                     int secs, int nsecs, bool store_values)
{

  Eigen::Vector2f ego(ego_x, ego_y);

  // STATE
  Eigen::Vector2f delta_state_size = Eigen::Vector2f::Zero();
  Eigen::Vector2f new_state_size(length_to_plot, width_to_plot);
  Eigen::Vector2f new_state_centroid = X.head(2).cast<float>();
  float growing_factor = 1.0; //0.5;

  // delta state
  if (length_t > length_to_plot)
    delta_state_size(0) = growing_factor * (length_t - length_to_plot);

  if (width_t > width_to_plot)
    delta_state_size(1) = growing_factor * (width_t - width_to_plot);

  if (delta_state_size.squaredNorm() > 0)
  {
    // find closest vertex & delta quadrant
    Eigen::Vector2f a, b, c, d;
    a << -length_to_plot / 2, width_to_plot / 2;
    b << length_to_plot / 2, width_to_plot / 2;
    c << length_to_plot / 2, -width_to_plot / 2;
    d << -length_to_plot / 2, -width_to_plot / 2;

    Eigen::Rotation2Df rotation_matrix = Eigen::Rotation2Df(X(2));
    a = rotation_matrix * a + X.head(2).cast<float>();
    b = rotation_matrix * b + X.head(2).cast<float>();
    c = rotation_matrix * c + X.head(2).cast<float>();
    d = rotation_matrix * d + X.head(2).cast<float>();

    Eigen::Vector2f closest_vertex = a;
    Eigen::Matrix2f sign = Eigen::Matrix2f::Identity();
    sign(0, 0) = 1;
    sign(1, 1) = -1;

    if ((b - ego).squaredNorm() < (closest_vertex - ego).squaredNorm())
    {
      closest_vertex = b;
      sign(0, 0) = -1;
      sign(1, 1) = -1;
    }
    if ((c - ego).squaredNorm() < (closest_vertex - ego).squaredNorm())
    {
      closest_vertex = c;
      sign(0, 0) = -1;
      sign(1, 1) = 1;
    }
    if ((d - ego).squaredNorm() < (closest_vertex - ego).squaredNorm())
    {
      closest_vertex = d;
      sign(0, 0) = 1;
      sign(1, 1) = 1;
    }

    // new size
    new_state_size += delta_state_size;

    // new centroid
    new_state_centroid += rotation_matrix * (sign * delta_state_size / 2);

    // update if store
    if (store_values)
    {
      length_to_plot = new_state_size(0);
      width_to_plot = new_state_size(1);
      X.head(2) = new_state_centroid.cast<double>();
    }
  }

  // MEASUREMENT
  Eigen::Vector2f delta_meas_size = Eigen::Vector2f::Zero();
  Eigen::Vector2f meas_size(length_t, width_t);
  Eigen::Vector2f meas_centroid(xt, yt);

  // delta state
  delta_meas_size = new_state_size - meas_size;

  // find closest vertex & delta quadrant
  Eigen::Vector2f a, b, c, d;
  a << -length_t / 2, width_t / 2;
  b << length_t / 2, width_t / 2;
  c << length_t / 2, -width_t / 2;
  d << -length_t / 2, -width_t / 2;

  Eigen::Rotation2Df rotation_matrix = Eigen::Rotation2Df(theta_t);
  a = rotation_matrix * a + meas_centroid;
  b = rotation_matrix * b + meas_centroid;
  c = rotation_matrix * c + meas_centroid;
  d = rotation_matrix * d + meas_centroid;

  Eigen::Vector2f closest_vertex = a;
  Eigen::Matrix2f sign = Eigen::Matrix2f::Identity();
  sign(0, 0) = 1;
  sign(1, 1) = -1;

  if (b.squaredNorm() < closest_vertex.squaredNorm())
  {
    closest_vertex = b;
    sign(0, 0) = -1;
    sign(1, 1) = -1;
  }
  if (c.squaredNorm() < closest_vertex.squaredNorm())
  {
    closest_vertex = c;
    sign(0, 0) = -1;
    sign(1, 1) = 1;
  }
  if (d.squaredNorm() < closest_vertex.squaredNorm())
  {
    closest_vertex = d;
    sign(0, 0) = 1;
    sign(1, 1) = 1;
  }

  // new size
  Eigen::Vector2f new_meas_size = meas_size + delta_meas_size;

  // new centroid
  Eigen::Vector2f new_meas_centroid = meas_centroid + rotation_matrix * (sign * delta_meas_size / 2);

  // Correcting observation using the geometry
  length_t = new_meas_size(0);
  width_t = new_meas_size(1);
  xt = new_meas_centroid(0);
  yt = new_meas_centroid(1);

}

void TrackingTarget::CalculateStateJacobian(double dt)
{
  //f(x)
  F_X(0, 0) = 1.0;
  F_X(0, 1) = 0.0;
  F_X(0, 2) = -X(3, 0) * sin(X(2, 0)) * dt;
  F_X(0, 3) = cos(X(2, 0)) * dt;
  F_X(0, 4) = 0.0;

  //f(y)
  F_X(1, 0) = 0.0;
  F_X(1, 1) = 1.0;
  F_X(1, 2) = X(3, 0) * cos(X(2, 0)) * dt;
  F_X(1, 3) = sin(X(2, 0)) * dt;
  F_X(1, 4) = 0.0;

  //f(theta)
  F_X(2, 0) = 0.0;
  F_X(2, 1) = 0.0;
  F_X(2, 2) = 1.0;
  F_X(2, 3) = X(4) * dt;
  F_X(2, 4) = X(3) * dt;

  //f(v)
  F_X(3, 0) = 0.0;
  F_X(3, 1) = 0.0;
  F_X(3, 2) = 0.0;
  F_X(3, 3) = 1.0;
  F_X(3, 4) = 0.0;

  //f(omega)
  F_X(4, 0) = 0.0;
  F_X(4, 1) = 0.0;
  F_X(4, 2) = 0.0;
  F_X(4, 3) = 0.0;
  F_X(4, 4) = 1.0;

//  std::cout << "TrackingTarget::CalculateStateJacobian --> F_X = " << std::endl << F_X << std::endl;
//  std::cout << "Delta t = " << dt << std::endl;
}

float TrackingTarget::Update(CObservation obs, float theta_noise_factor, float ego_x, float ego_y, float ego_theta,
                             bool associated)
{
  const float INVALID_LIKELIHOOD = -1.0;

  float x = 0.0, y = 0.0, theta = 0.0, width = 0.0, length = 0.0;
  int id = 0, secs = 0, nsecs = 0;
  float prob = 0.0;

  float mean_squared_error = 0.0;

  float likelihood = INVALID_LIKELIHOOD;

  obs.Get(x, y, theta, mean_squared_error, width, length, secs, nsecs, id, prob);
  likelihood = Update(ego_x, ego_y, ego_theta, x, y, width, length, theta, theta_noise_factor,
                                mean_squared_error, secs, nsecs, associated);

  return (likelihood);
}

float TrackingTarget::Update(float ego_x, float ego_y, float ego_theta, float xt, float yt, float width_t,
                             float length_t, float theta_t, float theta_noise_factor, float mean_squared_error,
                             int secs, int nsecs, bool associated)
{
  double delta_t = 0.0; // Increment of time since last update

  const float MAHALANOBIS_THRESHOLD = 3.0;
  const float INVALID_DISTANCE = -1.0;
  float mahalanobis_distance = INVALID_DISTANCE;
  float likelihood = INVALID_DISTANCE;

  Xs(0) = ego_x;
  Xs(1) = ego_y;
  Xs(2) = ego_theta;

  Rot = Eigen::Matrix3d::Identity();
  Rot.topLeftCorner<2, 2>() = Eigen::Rotation2Dd(-Xs(2)).matrix();

  if (associated)
  {
    width = width_t;
    length = length_t;
  }

  if (iteration_number > 0) // We can compute delta_t
  {
    delta_t = secs - t_secs;
    delta_t = delta_t + (nsecs - t_nsecs) * 1e-9; // Expresing the timestamp as an increment since last update

    // We want to convert the x and y coordinates observed to express the value of the centroid of the
    // cluster with the expected dimensions, not only the visible dimensions --> it turns the x and y observed
    // into the center of the bounding box
    if (associated)
      CorrectingObservationByGeometry(xt, yt, width_t, length_t, theta_t, ego_x, ego_y, ego_theta, secs, nsecs, true);

    // Initializing variables
    y(0) = xt; //Filling the observation vector
    y(1) = yt;
    y(2) = theta_t;

  }
  else
  {
    // First iteration

    //Initializing variables
    initial_width = width_t;
    initial_length = length_t;
    width_to_plot = width_t;
    length_to_plot = length_t;

    y(0) = xt; //Filling the observation vector
    y(1) = yt;
    y(2) = theta_t;

    // Initializating the state with the observed values in world coordinates
    X.head<3>() = Xs + Rot.transpose() * y;
  }

  CalculateStateJacobian(delta_t);

  //Initializing velocities in the second iteration
  if (iteration_number == 1)
  {
    Eigen::Vector3d observation_in_world_frame = Xs + Rot.transpose() * y;

    X(3) = sqrt(
        ((observation_in_world_frame(0) - X(0)) / delta_t) * ((observation_in_world_frame(0) - X(0)) / delta_t)
            + ((observation_in_world_frame(1) - X(1)) / delta_t) * ((observation_in_world_frame(1) - X(1)) / delta_t));

    X(4) = 0.0;
  }

  t_secs = secs;   // Updating time (losing values of t-1)
  t_nsecs = nsecs;

  // State prediction
  // x coordinate
  X(0) = X(0) + X(3) * delta_t * cos(X(2));

  // y coordinate
  X(1) = X(1) + X(3) * delta_t * sin(X(2));

  //Box orientation
  X(2) = X(2) + X(3) * X(4) * delta_t;

  //Linear velocity (constant) X(3) = X(3);
  //K (constant) X(4) = X(4);
  //std::cout << "TrackingTarget::Update --> delta_t = " << delta_t << std::endl;
  //std::cout << "TrackingTarget::Update --> Predicted X: " << std::endl << X.transpose() << std::endl;

//  std::cout << "TrackingTarget::Update P: " << P << std::endl;
//  std::cout << "TrackingTarget::Update Q: " << Q << std::endl;
//  std::cout << "TrackingTarget::Update F_X: " << F_X << std::endl;
//  std::cout << "TrackingTarget::Update F_q: " << F_q << std::endl;

  // Covariance prediction
  P = F_X * P * F_X.transpose() + F_q * Q * F_q.transpose();

  //std::cout << "TrackingTarget::Update --> Predicted P: " << std::endl << P << std::endl;

  //if(iteration_number == 0) std::cout << "P:\n" << P << std::endl;

  if (associated)
  {
    // Expectation
    e = Rot * (X.head<3>() - Xs);

    // Jacobian of the observation function
    H.block<3, 3>(0, 0) = Rot;
    H.block<3, 2>(0, 3).setZero();
//    std::cout << "TrackingTarget::Update H: " << H << std::endl;

    // Innovation
    z = y - e;

    // Innovation covariance

    // Applying the noise factor that relates the fitting error and the dimensions
    // of the observed object with the orientation error, it is calculated in Track.cpp
    R(2, 2) = R(2, 2) * theta_noise_factor;

    Z = H * P * H.transpose() + R;

    // After using this correction it is needed to revert the changes because the fitting
    // error changes each iteration
    R(2, 2) = R(2, 2) / theta_noise_factor;

//    std::cout << "TrackingTarget::Update R: " << R << std::endl;

    mahalanobis_distance = sqrt(z.transpose() * Z.inverse() * z);

    likelihood = exp(-0.5 * mahalanobis_distance) / sqrt(Z.determinant());

//    std::cout << "TrackingTarget::Update mahalanobis_distance: " << mahalanobis_distance << std::endl;

    if (mahalanobis_distance < MAHALANOBIS_THRESHOLD)
    {
      // Kalman gain
      K = P * H.transpose() * Z.inverse();
//      std::cout << "TrackingTarget::Update K: " << K << std::endl;

      // State correction
      X = X + K * z;
//      std::cout << "TrackingTarget::Update X: " << X << std::endl;

      // State covariance correction
      P = P - K * Z * K.transpose();
      //std::cout << "TrackingTarget::Update --> Corrected P: " << std::endl << P << std::endl;
    }
  }
  iteration_number++;
  return (likelihood);

}

float TrackingTarget::Likelihood(float x_obs, float y_obs, float theta_obs, float theta_noise_factor, float ego_x,
                                 float ego_y, float ego_theta, int secs, int nsecs)
{
  const float INVALID_DISTANCE = -1.0;
  float mahalanobis_distance = INVALID_DISTANCE;
  float likelihood = INVALID_DISTANCE;

  float x_pred, y_pred, theta_pred;
  Eigen::Vector3d X_pred;

  Eigen::Matrix<double, 5, 5> pred_cov = F_X * P * F_X.transpose() + F_q * Q * F_q.transpose();

  y(0) = x_obs;
  y(1) = y_obs;
  y(2) = theta_obs;

  GetExpectedObservation(x_pred, y_pred, theta_pred, ego_x, ego_y, ego_theta, secs, nsecs);

  e(0) = x_pred;
  e(1) = y_pred;
  e(2) = theta_pred;

  Rot = Eigen::Matrix3d::Identity();
  Rot.topLeftCorner<2, 2>() = Eigen::Rotation2Dd(-ego_theta).matrix();

  // Jacobian of the observation function
  H.block<3, 3>(0, 0) = Rot;
  H.block<3, 2>(0, 3).setZero();

  // Innovation
  z = y - e;

  // Innovation covariance

  // Applying the noise factor that relates the fitting error and the dimensions
  // of the observed object with the orientation error, it is calculated in Track.cpp
  R(2, 2) = R(2, 2) * theta_noise_factor;

  Z = H * pred_cov * H.transpose() + R;

  // After using this correction it is needed to revert the changes because the fitting
  // error changes each iteration
  R(2, 2) = R(2, 2) / theta_noise_factor;

//  std::cout << "TrackingTarget::Likelihood R: " << R << std::endl;

  mahalanobis_distance = sqrt(z.transpose() * Z.inverse() * z);

  likelihood = exp(-0.5 * mahalanobis_distance) / sqrt(Z.determinant());

//  std::cout << "Likelihood calculated" << std::endl;

  return (likelihood);
}

float TrackingTarget::MahalanobisDistance(float x_obs, float y_obs, float theta_obs, float theta_noise_factor,
                                          float ego_x, float ego_y, float ego_theta, int secs, int nsecs)
{
  const float INVALID_DISTANCE = -1.0;
  float mahalanobis_distance = INVALID_DISTANCE;

  float x_pred, y_pred, theta_pred;

  assert (iteration_number > 0 && "ERROR: in TrackingTarget::MahalanobisDistance, tried to compute with an unitialised track!");
  //Eigen::Vector3d X_pred;

//  std::cout << "TrackingTarget::MahalanobisDistance --> F_X = " << std::endl << F_X << std::endl;
//  std::cout << "TrackingTarget::MahalanobisDistance --> P = " << std::endl << P << std::endl;
//
//  std::cout << "TrackingTarget::MahalanobisDistance --> F_q = " << std::endl << F_q << std::endl;
//  std::cout << "TrackingTarget::MahalanobisDistance --> Q = " << std::endl << Q << std::endl;

  Eigen::Matrix<double, 5, 5> pred_cov = F_X * P * F_X.transpose() + F_q * Q * F_q.transpose();

//  std::cout << "TrackingTarget::MahalanobisDistance --> pred_cov, P = " << std::endl << pred_cov << std::endl;

  y(0) = x_obs;
  y(1) = y_obs;
  y(2) = theta_obs;

//  std::cout << "TrackingTarget::MahalanobisDistance --> observation, y = " << std::endl << y << std::endl;

  GetExpectedObservation(x_pred, y_pred, theta_pred, ego_x, ego_y, ego_theta, secs, nsecs);

  e(0) = x_pred;
  e(1) = y_pred;
  e(2) = theta_pred;

//  std::cout << "TrackingTarget::MahalanobisDistance --> expected observation, e = " << std::endl << e << std::endl;

  Rot = Eigen::Matrix3d::Identity();
  Rot.topLeftCorner<2, 2>() = Eigen::Rotation2Dd(-ego_theta).matrix();

  // Jacobian of the observation function
  H.block<3, 3>(0, 0) = Rot;
  H.block<3, 2>(0, 3).setZero();

//  std::cout << "TrackingTarget::MahalanobisDistance --> Jacobian of the observation function, H = " << std::endl << H << std::endl;

  // Innovation
  z = y - e;

//  std::cout << "TrackingTarget::MahalanobisDistance --> Innovation z = " << std::endl << z << std::endl;

  // Innovation covariance

  // Applying the noise factor that relates the fitting error and the dimensions
  // of the observed object with the orientation error, it is calculated in Track.cpp

//  std::cout << "TrackingTarget::MahalanobisDistance --> Measure additive noise not corrected R  = " << std::endl << R << std::endl;

  R(2, 2) = R(2, 2) * theta_noise_factor;

//  std::cout << "TrackingTarget::MahalanobisDistance --> Measure additive noise corrected, R  = " << std::endl << R << std::endl;

  Z = H * pred_cov * H.transpose() + R;

//  std::cout << "TrackingTarget::MahalanobisDistance --> Innovation covariance  Z = " << std::endl << Z << std::endl;

  // After using this correction it is needed to revert the changes because the fitting
  // error changes each iteration
  R(2, 2) = R(2, 2) / theta_noise_factor;
//  std::cout << "TrackingTarget::MahalanobisDistance --> Measure additive noise recovered R  = " << std::endl << R << std::endl;

  mahalanobis_distance = sqrt(z.transpose() * Z.inverse() * z);

  return (mahalanobis_distance);
}

float TrackingTarget::EuclideanDistance(float x_obs, float y_obs, float theta_obs, float ego_x, float ego_y,
                                        float ego_theta, int secs, int nsecs)
{
  float euclidean_distance = 0.0;

  float x_pred, y_pred, theta_pred;

  GetExpectedObservation(x_pred, y_pred, theta_pred, ego_x, ego_y, ego_theta, secs, nsecs);

  euclidean_distance = sqrt((x_pred - x_obs) * (x_pred - x_obs) + (y_pred - y_obs) * (y_pred - y_obs));

  return (euclidean_distance);
}

void TrackingTarget::GetExpectedObservation(float& x_req, float& y_req, float& theta_req, float ego_x, float ego_y,
                                            float ego_theta, int secs, int nsecs)
{

  double delta_t = 0.0;

  delta_t = secs - t_secs;
  delta_t = delta_t + (nsecs - t_nsecs) * 1e-9;

  // predicted x global coordinate
  Xe(0) = X(0) + X(3) * delta_t * cos(X(2));

  // predicted y global coordinate
  Xe(1) = X(1) + X(3) * delta_t * sin(X(2));

  // predicted theta in global coordinates
  Xe(2) = X(2) + X(3) * X(4) * delta_t;

  Xs(0) = ego_x;
  Xs(1) = ego_y;
  Xs(2) = ego_theta;

  Rot = Eigen::Matrix3d::Identity();
  Rot.topLeftCorner<2, 2>() = Eigen::Rotation2Dd(-Xs(2)).matrix();

  // Expectation
  Xe = Rot * (Xe - Xs);

  x_req = Xe(0);
  y_req = Xe(1);
  theta_req = Xe(2);

}

void TrackingTarget::GetDimensions(float& a, float& b)
{
  a = width_to_plot;
  b = length_to_plot;
}

void TrackingTarget::GetVariablesToPlot(float& x_req, float& y_req, float& w_req, float& l_req, float& theta_req,
                                        float& vel_req, float ego_x, float ego_y, float ego_theta)
{

  Xs(0) = ego_x;
  Xs(1) = ego_y;
  Xs(2) = ego_theta;

  Rot = Eigen::Matrix3d::Identity();
  Rot.topLeftCorner<2, 2>() = Eigen::Rotation2Dd(-Xs(2)).matrix();

  // Expectation
  Xe = Rot * (X.head<3>() - Xs);

  x_req = Xe(0);
  y_req = Xe(1);
  theta_req = Xe(2);
  w_req = width_to_plot;
  l_req = length_to_plot;
  vel_req = X(3);

}

void TrackingTarget::GetCovariance(float& cov_x, float& cov_y)
{
  cov_x = P(0, 0);
  cov_y = P(1, 1);
}

void TrackingTarget::GetFullStateAndCovariance(Eigen::Matrix<double, 5, 1>& state,
                                               Eigen::Matrix<double, 5, 5>& covariance)
{
  state = X;
  covariance = P;
}

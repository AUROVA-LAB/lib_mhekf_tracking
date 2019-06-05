#ifndef _struct_definitions_h_
#define _struct_definitions_h_

#include "observation.h"

namespace mhekf_tracker
{
struct EgoPose
{
  double x;
  double y;
  double theta;
};

struct TrackManagementConfiguration
{
  double max_covariance;
  double mahalanobis_threshold;
  int filter_iterations_before_allowing_update_without_observation;
};

struct KalmanConfiguration
{
  float x_ini, y_ini, theta_ini, v_ini, k_ini;
  float v_model, k_model;
  float x_sensor, y_sensor, theta_sensor;
  float mean_squared_error_noise_factor;
};

struct DoubleHypothesisObservation
{
  CObservation a;
  CObservation b;
  int id;
};

}
#endif

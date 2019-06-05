/*
 * Track.h

 *
 *  Created on: Dec 7, 2016
 *      Author: idelpino
 */

#ifndef INCLUDE_TRACK_H_
#define INCLUDE_TRACK_H_

#include "struct_definitions.h"
#include "tracking_target.h"
#include "observation.h"

#include <Eigen/Dense>

class CTrack
{
private:
  TrackingTarget hypothesis_a;
  TrackingTarget hypothesis_b;

  float probability_of_a;
  float probability_of_b;
  float best_hypothesis_probability;

  float obs_mean_squared_error;
  float obs_mean_objectness;

  int id;
  int iteration_number;
  int iterations_without_association;

  float z;
  float height;

  bool kill_flag;
  bool best_hypothesis_is_a;
  bool flag_only_one_hypothesis_remaining;

public:

  float likelihood;

  CTrack();

  void SetStandardDeviations(mhekf_tracker::KalmanConfiguration kalman_configuration);

  void SetId(int new_id);

  int GetId(void);

  void GetIterationNumber(int& iter_req);

  int GetIterationNumber(void);

  void GetVariablesToPlot(float& x_req, float& y_req, float& w_req, float& l_req, float& theta_req, float& vel_req,
                          float ego_x, float ego_y, float ego_theta);

  void GetVariablesToPlot(float& x_req, float& y_req, float& z_req, float& w_req, float& l_req, float& h_req,
                          float& theta_req, float& vel_req, float& fitting_error_req, float& mixture_req, float ego_x,
                          float ego_y, float ego_theta);

  float MahalanobisDistance(CObservation& obs, float mean_squared_error_noise_factor, float ego_x, float ego_y,
                            float ego_theta);

  float DoubleHypDoubleObsMahalanobisDistance(mhekf_tracker::DoubleHypothesisObservation dh_obs,
                                              float mean_squared_error_noise_factor, float ego_x, float ego_y,
                                              float ego_theta, bool hyp_a_match_obs_a, bool hyp_b_match_obs_b);

  //float Likelihood(CObservation& obs, float mean_squared_error_noise_factor, float ego_x, float ego_y, float ego_theta);

  void GetCovariance(float& cov_x, float& cov_y);

  void GetFullStateAndCovariance(Eigen::Matrix<double, 5, 1>& state, Eigen::Matrix<double, 5, 5>& covariance);

  void UpdateSingle(CObservation obs, float mean_squared_error_noise_factor, float ego_x, float ego_y, float ego_theta,
                    bool associated);

  void UpdateDouble(mhekf_tracker::DoubleHypothesisObservation dh_obs, float mean_squared_error_noise_factor,
                    float ego_x, float ego_y, float ego_theta, bool associated);

  void Update(mhekf_tracker::DoubleHypothesisObservation dh_obs, float mean_squared_error_noise_factor, float ego_x,
              float ego_y, float ego_theta, bool associated);

  void Update(CObservation obs, float mean_squared_error_noise_factor, float ego_x, float ego_y, float ego_theta,
              bool associated);

  void UpdateWithoutObservation(int secs, int nsecs);

  void SetKillFlag(void);

  bool GetKillFlag(void);

  int GetIterationsWithoutAssociation(void);

  bool GetHypothesis(void);

  bool GetFixedHypothesisFlag(void);

  void GetDimensions(float& a, float& b);

  void GetObsMeanSquaredError(float& omse);

  float GetBestHypothesisProbability(void)
  {
    return(best_hypothesis_probability);
  }

  virtual
  ~CTrack();

  void setBestHypothesisIsA(bool bestHypothesisIsA)
  {
    best_hypothesis_is_a = bestHypothesisIsA;
  }

  void setFlagOnlyOneHypothesisRemaining(bool flagOnlyOneHypothesisRemaining)
  {
    flag_only_one_hypothesis_remaining = flagOnlyOneHypothesisRemaining;
  }
};
#endif /* INCLUDE_TRACK_H_ */

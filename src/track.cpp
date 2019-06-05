/*
 * Track.cpp
 *
 *  Created on: Dec 7, 2016
 *      Author: idelpino
 */

#include "../include/track.h"
#include "struct_definitions.h"
#include "tracking_target.h"
#include "observation.h"

#include "math.h"

CTrack::CTrack()
{
  id = 0;

  iteration_number = 0;
  iterations_without_association = 0;

  obs_mean_squared_error = 0.0;
  obs_mean_objectness = 0.0;

  z = 0.0;
  height = 0.0;

  kill_flag = false;
  flag_only_one_hypothesis_remaining = false;
  best_hypothesis_is_a = true;

  probability_of_a = 0.5;
  probability_of_b = 0.5;

  best_hypothesis_probability = -1.0; // Initialising in impossible value
                                      // to detect unexpected situations

  likelihood = 0.0;
}

bool CTrack::GetHypothesis(void)
{
  return best_hypothesis_is_a;
}

CTrack::~CTrack()
{

}

void CTrack::SetStandardDeviations(mhekf_tracker::KalmanConfiguration kalman_configuration)
{
  hypothesis_a.SetStandardDeviations(kalman_configuration);
  hypothesis_b.SetStandardDeviations(kalman_configuration);
}

void CTrack::SetId(int new_id)
{
//  std::cout << "Changing trk ID: prev id = " << id << "    new id = " << new_id << std::endl;
  id = new_id;
}

int CTrack::GetId(void)
{
  return (id);
}

float CTrack::MahalanobisDistance(CObservation& obs, float mean_squared_error_noise_factor, float ego_x, float ego_y,
                                  float ego_theta)
{
  float distance = 0.0;
  float distance_a = 0.0;
  float distance_b = 0.0;

  float x_a, y_a, theta_a, width, length;
  float x_b, y_b, theta_b;

  float mean_squared_error = 0.0;

  int secs, nsecs, id;

  if (!flag_only_one_hypothesis_remaining || (flag_only_one_hypothesis_remaining && best_hypothesis_is_a))
  {
    // Storing the observed values in local variables
    float prob = 0.0;
    obs.Get(x_a, y_a, theta_a, mean_squared_error, width, length, secs, nsecs, id, prob);

    // Applying the geometric corrections based on the shape of the already
    // tracked object (this corrections are done to avoid centroid jumps when
    // changes in the point of view modify the observed dimensions)
    //
    // Also we have to specify that we don't want to change the dimensions of
    // the tracked object because this is a check, no an update, so store_values
    // is set to false
    bool store_values = false;

    hypothesis_a.CorrectingObservationByGeometry(x_a, y_a, width, length, theta_a, ego_x, ego_y, ego_theta, secs, nsecs,
                                                 store_values);

    float theta_noise_factor = mean_squared_error * mean_squared_error_noise_factor
        / ((width + length) * (width + length));

    distance_a = hypothesis_a.MahalanobisDistance(x_a, y_a, theta_a, theta_noise_factor, ego_x, ego_y, ego_theta, secs,
                                                  nsecs);
  }

  if (!flag_only_one_hypothesis_remaining || (flag_only_one_hypothesis_remaining && !best_hypothesis_is_a))
  {
    // Same process for the other hypothesis
    float prob = 0.0;
    obs.Get(x_b, y_b, theta_b, mean_squared_error, width, length, secs, nsecs, id, prob);

    // but changing the observed theta to take into account the ambiguity
    theta_b += M_PI / 2.0;

    // And flipping the dimensions
    float swap = width;
    width = length;
    length = swap;

    bool store_values = false;

    hypothesis_b.CorrectingObservationByGeometry(x_b, y_b, width, length, theta_b, ego_x, ego_y, ego_theta, secs, nsecs,
                                                 store_values);

    float theta_noise_factor = mean_squared_error * mean_squared_error_noise_factor
        / ((width + length) * (width + length));

    distance_b = hypothesis_b.MahalanobisDistance(x_b, y_b, theta_b, theta_noise_factor, ego_x, ego_y, ego_theta, secs,
                                                  nsecs);
  }

  if (!flag_only_one_hypothesis_remaining)
  {
    if (distance_a < distance_b)
    {
      distance = distance_a;
    }
    else
    {
      distance = distance_b;
    }
  }
  else
  {
    if (best_hypothesis_is_a)
    {
      distance = distance_a;
    }
    else
    {
      distance = distance_b;
    }
  }
  return (distance);
}

float CTrack::DoubleHypDoubleObsMahalanobisDistance(mhekf_tracker::DoubleHypothesisObservation dh_obs,
                                                    float mean_squared_error_noise_factor, float ego_x, float ego_y,
                                                    float ego_theta, bool hyp_a_match_obs_a, bool hyp_b_match_obs_b)
{
  float distance = 10000.0;
  float distance_a_a = 10000.0;
  float distance_a_b = 10000.0;
  float distance_b_a = 10000.0;
  float distance_b_b = 10000.0;

  float x_a, y_a, theta_a, width_a, length_a;
  float x_b, y_b, theta_b, width_b, length_b;

  float mean_squared_error_a = 0.0;
  float mean_squared_error_b = 0.0;

  float theta_noise_factor_a = 0.0;
  float theta_noise_factor_b = 0.0;

  float obs_prob_a = 0.0;
  float obs_prob_b = 0.0;

  int secs, nsecs, id;

  int obs_id = 0;

//  std::cout << "Calculating Mahalanobis distance of dh_obs with ID = " << dh_obs.a.GetId() << std::endl;

  assert(
      iteration_number > 0 && "ERROR: in CTrack::MahalanobisDistance, tried to compute with an uninitialised track!");

  // Storing the observed values in local variables

//  std::cout << "Using Observation Hyp. A, with P(A) = " << dh_obs.a.getProbability() << std::endl;
  dh_obs.a.Get(x_a, y_a, theta_a, mean_squared_error_a, width_a, length_a, secs, nsecs, obs_id, obs_prob_a);
  theta_noise_factor_a = mean_squared_error_a * mean_squared_error_noise_factor
      / ((width_a + length_a) * (width_a + length_a));

//  std::cout << "Using Observation Hyp. B, with P(B) = " << dh_obs.b.getProbability() << std::endl;
  dh_obs.b.Get(x_b, y_b, theta_b, mean_squared_error_b, width_b, length_b, secs, nsecs, obs_id, obs_prob_b);
  theta_noise_factor_b = mean_squared_error_b * mean_squared_error_noise_factor
      / ((width_b + length_b) * (width_b + length_b));

  if (!flag_only_one_hypothesis_remaining || (flag_only_one_hypothesis_remaining && best_hypothesis_is_a))
  {
    // Applying the geometric corrections based on the shape of the already
    // tracked object (this corrections are done to avoid centroid jumps when
    // changes in the point of view modify the observed dimensions)
    //
    // Also we have to specify that we don't want to change the dimensions of
    // the tracked object because this is a check, no an update, so store_values
    // is set to false
    bool store_values = false;

    hypothesis_a.CorrectingObservationByGeometry(x_a, y_a, width_a, length_a, theta_a, ego_x, ego_y, ego_theta, secs,
                                                 nsecs, store_values);

    distance_a_a = hypothesis_a.MahalanobisDistance(x_a, y_a, theta_a, theta_noise_factor_a, ego_x, ego_y, ego_theta,
                                                    secs, nsecs);

//    std::cout << "Computing mahab distance: Trk Hyp A --> Obs Hyp A.............. Distance = " << distance_a_a
//        << std::endl;

    hypothesis_a.CorrectingObservationByGeometry(x_b, y_b, width_b, length_b, theta_b, ego_x, ego_y, ego_theta, secs,
                                                 nsecs, store_values);
    distance_a_b = hypothesis_a.MahalanobisDistance(x_b, y_b, theta_b, theta_noise_factor_b, ego_x, ego_y, ego_theta,
                                                    secs, nsecs);
//    std::cout << "Computing mahab distance: Trk Hyp A --> Obs Hyp B.............. Distance = " << distance_a_b
//        << std::endl;
  }

  if (!flag_only_one_hypothesis_remaining || (flag_only_one_hypothesis_remaining && !best_hypothesis_is_a))
  {
    bool store_values = false;

    hypothesis_b.CorrectingObservationByGeometry(x_a, y_a, width_a, length_a, theta_a, ego_x, ego_y, ego_theta, secs,
                                                 nsecs, store_values);
    distance_b_a = hypothesis_b.MahalanobisDistance(x_a, y_a, theta_a, theta_noise_factor_a, ego_x, ego_y, ego_theta,
                                                    secs, nsecs);

//    std::cout << "Computing mahab distance: Trk Hyp B --> Obs Hyp A.............. Distance = " << distance_b_a
//        << std::endl;

    hypothesis_b.CorrectingObservationByGeometry(x_b, y_b, width_b, length_b, theta_b, ego_x, ego_y, ego_theta, secs,
                                                 nsecs, store_values);
    distance_b_b = hypothesis_b.MahalanobisDistance(x_b, y_b, theta_b, theta_noise_factor_b, ego_x, ego_y, ego_theta,
                                                    secs, nsecs);

//    std::cout << "Computing mahab distance: Trk Hyp B --> Obs Hyp B.............. Distance = " << distance_b_b
//        << std::endl;
  }

  hyp_a_match_obs_a = false;
  if (distance_a_a < distance_a_b)
  {
    hyp_a_match_obs_a = true;
//    std::cout << "Trk hyp A matches Obs hyp A!" << std::endl;
  }
  else
  {
//    std::cout << "Trk hyp A matches Obs hyp B!" << std::endl;
  }

  hyp_b_match_obs_b = false;
  if (distance_b_b < distance_b_a)
  {
    hyp_b_match_obs_b = true;
//    std::cout << "Trk hyp B matches Obs hyp A!" << std::endl;
  }
  else
  {
//    std::cout << "Trk hyp B matches Obs hyp B!" << std::endl;
  }

/////////////////////////////
  if (distance_a_a < distance)
    distance = distance_a_a;

  if (distance_a_b < distance)
    distance = distance_a_b;

  if (distance_b_a < distance)
    distance = distance_b_a;

  if (distance_b_b < distance)
    distance = distance_b_b;

//  std::cout << "distance_a_a = " << distance_a_a << "    distance_a_b = " << distance_a_b << "    distance_b_a = "
//      << distance_b_a << "     distance_b_b = " << distance_b_b << std::endl;

//  std::cout << "minimum distance = " << distance << std::endl;

  return (distance);
}

void CTrack::GetCovariance(float& cov_x, float& cov_y)
{
  if (best_hypothesis_is_a)
  {
    hypothesis_a.GetCovariance(cov_x, cov_y);
  }
  else
  {
    hypothesis_b.GetCovariance(cov_x, cov_y);
  }
//  std::cout << "Track ID = " << id << std::endl;
//  std::cout << "cov_x = " << cov_x << "    cov_y = " << cov_y << std::endl;
}

void CTrack::GetFullStateAndCovariance(Eigen::Matrix<double, 5, 1>& state, Eigen::Matrix<double, 5, 5>& covariance)
{
  if (best_hypothesis_is_a)
  {
    hypothesis_a.GetFullStateAndCovariance(state, covariance);
  }
  else
  {
    hypothesis_b.GetFullStateAndCovariance(state, covariance);
  }
}

void CTrack::GetVariablesToPlot(float& x_req, float& y_req, float& w_req, float& l_req, float& theta_req,
                                float& vel_req, float ego_x, float ego_y, float ego_theta)
{
  if (best_hypothesis_is_a)
  {
    hypothesis_a.GetVariablesToPlot(x_req, y_req, w_req, l_req, theta_req, vel_req, ego_x, ego_y, ego_theta);

    //std::cout << "Plotting hypothesis A" << std::endl;

  }
  else
  {
    hypothesis_b.GetVariablesToPlot(x_req, y_req, w_req, l_req, theta_req, vel_req, ego_x, ego_y, ego_theta);

    //std::cout << "Plotting hypothesis B" << std::endl;
  }
}

void CTrack::GetVariablesToPlot(float& x_req, float& y_req, float& z_req, float& w_req, float& l_req, float& h_req,
                                float& theta_req, float& vel_req, float& fitting_error_req, float& mixture_req, float ego_x, float ego_y,
                                float ego_theta)
{
  z_req = z;
  h_req = height;
  fitting_error_req = obs_mean_squared_error;
  mixture_req = obs_mean_objectness;
  if (best_hypothesis_is_a)
  {
    hypothesis_a.GetVariablesToPlot(x_req, y_req, w_req, l_req, theta_req, vel_req, ego_x, ego_y, ego_theta);

    //std::cout << "Plotting hypothesis A" << std::endl;

  }
  else
  {
    hypothesis_b.GetVariablesToPlot(x_req, y_req, w_req, l_req, theta_req, vel_req, ego_x, ego_y, ego_theta);

    //std::cout << "Plotting hypothesis B" << std::endl;
  }
}

void CTrack::UpdateSingle(CObservation obs, float mean_squared_error_noise_factor, float ego_x, float ego_y,
                          float ego_theta, bool associated)
{
  const float IMPOSSIBLE_LIKELIHOOD = -1.0;
  const float PROB_THRESHOLD = 0.001;

  float likelihood_a = IMPOSSIBLE_LIKELIHOOD;
  float x_a = 0.0, y_a = 0.0, theta_a = 0.0, width_a = 0.0, length_a = 0.0;
  float mean_squared_error_a = 0.0;

  int secs = 0, nsecs = 0, obs_id = 0;
  float prob = 0;

  obs.Get(x_a, y_a, theta_a, mean_squared_error_a, width_a, length_a, secs, nsecs, obs_id, prob);

  assert(prob > 0.999 && prob < 1.001 && "Error in CTrack::UpdateSingle, attempted to use a not single observation!");

  float theta_noise_factor_a = mean_squared_error_a * mean_squared_error_noise_factor
      / ((width_a + length_a) * (width_a + length_a));

  if (iteration_number == 0)
  {
    flag_only_one_hypothesis_remaining = true;
    best_hypothesis_is_a = true;
    likelihood = hypothesis_a.Update(ego_x, ego_y, ego_theta, x_a, y_a, width_a, length_a, theta_a,
                                     theta_noise_factor_a, mean_squared_error_a, secs, nsecs, associated);
  }
  else
  {
    if (flag_only_one_hypothesis_remaining)
    {
      if (best_hypothesis_is_a)
      {
        likelihood = hypothesis_a.Update(ego_x, ego_y, ego_theta, x_a, y_a, width_a, length_a, theta_a,
                                         theta_noise_factor_a, mean_squared_error_a, secs, nsecs, associated);
      }
      else
      {
        likelihood = hypothesis_b.Update(ego_x, ego_y, ego_theta, x_a, y_a, width_a, length_a, theta_a,
                                         theta_noise_factor_a, mean_squared_error_a, secs, nsecs, associated);
      }
    }
    else
    {
      flag_only_one_hypothesis_remaining = true;

      bool store_values = false;
      hypothesis_a.CorrectingObservationByGeometry(x_a, y_a, width_a, length_a, theta_a, ego_x, ego_y, ego_theta, secs,
                                                   nsecs, store_values);

      float distance_a_a = hypothesis_a.MahalanobisDistance(x_a, y_a, theta_a, theta_noise_factor_a, ego_x, ego_y,
                                                            ego_theta, secs, nsecs);

      obs.Get(x_a, y_a, theta_a, mean_squared_error_a, width_a, length_a, secs, nsecs, obs_id, prob);

      hypothesis_b.CorrectingObservationByGeometry(x_a, y_a, width_a, length_a, theta_a, ego_x, ego_y, ego_theta, secs,
                                                   nsecs, store_values);
      float distance_b_a = hypothesis_b.MahalanobisDistance(x_a, y_a, theta_a, theta_noise_factor_a, ego_x, ego_y,
                                                            ego_theta, secs, nsecs);

      obs.Get(x_a, y_a, theta_a, mean_squared_error_a, width_a, length_a, secs, nsecs, obs_id, prob);
      if (distance_a_a < distance_b_a)
      {
        best_hypothesis_is_a = true;
        likelihood = hypothesis_a.Update(ego_x, ego_y, ego_theta, x_a, y_a, width_a, length_a, theta_a,
                                         theta_noise_factor_a, mean_squared_error_a, secs, nsecs, associated);
      }
      else
      {
        best_hypothesis_is_a = false;
        likelihood = hypothesis_b.Update(ego_x, ego_y, ego_theta, x_a, y_a, width_a, length_a, theta_a,
                                         theta_noise_factor_a, mean_squared_error_a, secs, nsecs, associated);
      }
    }
  }
}

void CTrack::UpdateDouble(mhekf_tracker::DoubleHypothesisObservation dh_obs, float mean_squared_error_noise_factor,
                          float ego_x, float ego_y, float ego_theta, bool associated)
{
  const float IMPOSSIBLE_LIKELIHOOD = -1.0;
  const float PROB_THRESHOLD = 0.001;

  float likelihood_a = IMPOSSIBLE_LIKELIHOOD;
  float likelihood_b = IMPOSSIBLE_LIKELIHOOD;

  float x_a = 0.0, y_a = 0.0, theta_a = 0.0, width_a = 0.0, length_a = 0.0;
  float x_b = 0.0, y_b = 0.0, theta_b = 0.0, width_b = 0.0, length_b = 0.0;

  float mean_squared_error_a = 0.0;
  float mean_squared_error_b = 0.0;

  float theta_noise_factor_a = 0.0;
  float theta_noise_factor_b = 0.0;

  float obs_prob_a = 0.0;
  float obs_prob_b = 0.0;

  int secs = 0, nsecs = 0, obs_id = 0;

  bool use_obs_a = false;
  bool use_obs_b = false;

  bool hyp_a_match_obs_a = false;
  bool hyp_b_match_obs_b = false;

// Storing the observed values in local variables
  if (dh_obs.a.getProbability() > 0.001)
  {
    use_obs_a = true;
    dh_obs.a.Get(x_a, y_a, theta_a, mean_squared_error_a, width_a, length_a, secs, nsecs, obs_id, obs_prob_a);
    theta_noise_factor_a = mean_squared_error_a * mean_squared_error_noise_factor
        / ((width_a + length_a) * (width_a + length_a));
  }

  if (dh_obs.b.getProbability() > 0.001)
  {
    use_obs_b = true;
    dh_obs.b.Get(x_b, y_b, theta_b, mean_squared_error_b, width_b, length_b, secs, nsecs, obs_id, obs_prob_b);
    theta_noise_factor_b = mean_squared_error_b * mean_squared_error_noise_factor
        / ((width_b + length_b) * (width_b + length_b));
  }

  if (iteration_number == 0)
  {
    likelihood_a = hypothesis_a.Update(ego_x, ego_y, ego_theta, x_a, y_a, width_a, length_a, theta_a,
                                       theta_noise_factor_a, mean_squared_error_a, secs, nsecs, associated);

    likelihood_b = hypothesis_b.Update(ego_x, ego_y, ego_theta, x_b, y_b, width_b, length_b, theta_b,
                                       theta_noise_factor_b, mean_squared_error_b, secs, nsecs, associated);
  }
  else
  {
    float mah_dist = DoubleHypDoubleObsMahalanobisDistance(dh_obs, mean_squared_error_noise_factor, ego_x, ego_y,
                                                           ego_theta, hyp_a_match_obs_a, hyp_b_match_obs_b);

    // If we have to make an update of the trk hyp. A
    if (!flag_only_one_hypothesis_remaining || (flag_only_one_hypothesis_remaining && best_hypothesis_is_a))
    {
      if (hyp_a_match_obs_a)
      {
        likelihood_a = hypothesis_a.Update(ego_x, ego_y, ego_theta, x_a, y_a, width_a, length_a, theta_a,
                                           theta_noise_factor_a, mean_squared_error_a, secs, nsecs, associated);
      }
      else
      {
        likelihood_a = hypothesis_a.Update(ego_x, ego_y, ego_theta, x_b, y_b, width_b, length_b, theta_b,
                                           theta_noise_factor_b, mean_squared_error_b, secs, nsecs, associated);
      }
    }

    // If we have to make an update of the trk hyp. B
    if (!flag_only_one_hypothesis_remaining || (flag_only_one_hypothesis_remaining && !best_hypothesis_is_a))
    {
      if (hyp_b_match_obs_b)
      {
        likelihood_b = hypothesis_b.Update(ego_x, ego_y, ego_theta, x_b, y_b, width_b, length_b, theta_b,
                                           theta_noise_factor_b, mean_squared_error_b, secs, nsecs, associated);
      }
      else
      {
        likelihood_b = hypothesis_b.Update(ego_x, ego_y, ego_theta, x_a, y_a, width_a, length_a, theta_a,
                                           theta_noise_factor_a, mean_squared_error_a, secs, nsecs, associated);
      }
    }

    if (!flag_only_one_hypothesis_remaining)
    {
      float prev_a_prob = probability_of_a;
      float prev_b_prob = probability_of_b;

      probability_of_a = (likelihood_a * prev_a_prob) / ((likelihood_a * prev_a_prob) + (likelihood_b * prev_b_prob));
      probability_of_b = (likelihood_b * prev_b_prob) / ((likelihood_a * prev_a_prob) + (likelihood_b * prev_b_prob));

      // Including observation probability
      float p_a = probability_of_a * dh_obs.a.getProbability();
      float p_b = probability_of_b * dh_obs.b.getProbability();
      // Re-normalising
      probability_of_a = p_a / (p_a + p_b);
      probability_of_b = p_b / (p_a + p_b);

      assert(
          probability_of_a + probability_of_b > 0.999 && probability_of_a + probability_of_b < 1.001
              && "Error in CTrack::Update total prob != 1.0");

//      std::cout << "Track num = " << id << "    P(A) = " << probability_of_a << "    P(B) = " << probability_of_b
//          << std::endl;

      if (associated)
      {
        if (probability_of_a > probability_of_b)
        {
          best_hypothesis_is_a = true;
        }
        else
        {
          best_hypothesis_is_a = false;
        }
        if (probability_of_a < PROB_THRESHOLD || probability_of_b < PROB_THRESHOLD)
        {
          flag_only_one_hypothesis_remaining = true;
        }
      }
    }
  }
}

void CTrack::Update(mhekf_tracker::DoubleHypothesisObservation dh_obs, float mean_squared_error_noise_factor,
                    float ego_x, float ego_y, float ego_theta, bool associated)
{
  if (associated)
  {
    height = dh_obs.a.getHeight();
    z = dh_obs.a.getZ();
  }

  if (dh_obs.b.getProbability() < 0.001)
  {
    UpdateSingle(dh_obs.a, mean_squared_error_noise_factor, ego_x, ego_y, ego_theta, associated);
  }
  else
  {
    UpdateDouble(dh_obs, mean_squared_error_noise_factor, ego_x, ego_y, ego_theta, associated);
  }

  iteration_number++;
  if (!associated)
  {
    iterations_without_association++;
  }
  else
  {
    iterations_without_association = 0;
  }
}

void CTrack::Update(CObservation obs, float mean_squared_error_noise_factor, float ego_x, float ego_y, float ego_theta,
                    bool associated)
{
  const float IMPOSSIBLE_LIKELIHOOD = -1.0;
  const float PROB_THRESHOLD = 0.001;

  float likelihood_a = IMPOSSIBLE_LIKELIHOOD;
  float likelihood_b = IMPOSSIBLE_LIKELIHOOD;

  float x, y, theta, width, length;

  float mean_squared_error = 0.0;
  float mean_objectness = 0.0;

  int secs, nsecs, obs_id;

  // Storing the observed values in local variables
  float prob = 0.0;
  obs.Get(x, y, z, theta, mean_squared_error, mean_objectness, width, length, height, secs, nsecs, obs_id, prob);
  //std::cout << "Observation ID: " << obs_id << "    Observation mean objectness = " << mean_objectness << std::endl;

  float theta_noise_factor = mean_squared_error * mean_squared_error_noise_factor
      / ((width + length) * (width + length));

  //std::cout << "Track id = "<< id << "    noise factor = " << theta_noise_factor << std::endl;

  if (!flag_only_one_hypothesis_remaining || (flag_only_one_hypothesis_remaining && best_hypothesis_is_a))
  {
    std::cout << "Updating hypothesis A ...." << std::endl;
    likelihood_a = hypothesis_a.Update(ego_x, ego_y, ego_theta, x, y, width, length, theta, theta_noise_factor,
                                       mean_squared_error, secs, nsecs, associated);
    if (associated && length > 2.2)
    {
      best_hypothesis_is_a = true;
      best_hypothesis_probability = 1.0;
      flag_only_one_hypothesis_remaining = true;
    }
  }

  if (!flag_only_one_hypothesis_remaining || (flag_only_one_hypothesis_remaining && !best_hypothesis_is_a))
  {
    // Changing the observed theta to feed the second hypothesis
    theta += M_PI / 2.0;

    // And flipping the dimensions
    float swap = width;
    width = length;
    length = swap;
    std::cout << "Updating hypothesis B...." << std::endl;
    likelihood_b = hypothesis_b.Update(ego_x, ego_y, ego_theta, x, y, width, length, theta, theta_noise_factor,
                                       mean_squared_error, secs, nsecs, associated);

    if (associated && length > 2.2)
    {
      best_hypothesis_is_a = false;
      best_hypothesis_probability = 1.0;
      flag_only_one_hypothesis_remaining = true;
    }
  }

  if (!flag_only_one_hypothesis_remaining)
  {
    float prev_a_prob = probability_of_a;
    float prev_b_prob = probability_of_b;

    probability_of_a = (likelihood_a * prev_a_prob) / ((likelihood_a * prev_a_prob) + (likelihood_b * prev_b_prob));
    probability_of_b = (likelihood_b * prev_b_prob) / ((likelihood_a * prev_a_prob) + (likelihood_b * prev_b_prob));

    std::cout << "Track num = " << id << "    P(A) = " << probability_of_a << "    P(B) = " << probability_of_b << std::endl;

    if (associated)
    {
      if (probability_of_a > probability_of_b)
      {
        best_hypothesis_is_a = true;
        best_hypothesis_probability = probability_of_a;
      }
      else
      {
        best_hypothesis_is_a = false;
        best_hypothesis_probability = probability_of_b;
      }
      if (probability_of_a < PROB_THRESHOLD || probability_of_b < PROB_THRESHOLD)
      {
        flag_only_one_hypothesis_remaining = true;
      }
    }
  }

  iteration_number++;
  if (!associated)
  {
    iterations_without_association++;
  }
  else
  {
    obs_mean_squared_error = mean_squared_error;
    obs_mean_objectness = mean_objectness;
    iterations_without_association = 0;
  }

}

void CTrack::UpdateWithoutObservation(int secs, int nsecs)
{
  bool associated = false;

  if (!flag_only_one_hypothesis_remaining || best_hypothesis_is_a)
  {
    std::cout << "Updating hypothesis A without observation ...." << std::endl;
    hypothesis_a.Update(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, secs, nsecs, associated);
  }

  if (!flag_only_one_hypothesis_remaining || !best_hypothesis_is_a)
  {
    std::cout << "Updating hypothesis B without observation ...." << std::endl;
    hypothesis_b.Update(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, secs, nsecs, associated);
  }
  iterations_without_association++;
  iteration_number++;

}

void CTrack::GetObsMeanSquaredError(float& omse)
{
  omse = obs_mean_squared_error;
}

void CTrack::GetIterationNumber(int& iter_req)
{
  iter_req = iteration_number;
}

int CTrack::GetIterationsWithoutAssociation(void)
{
  return (iterations_without_association);
}

int CTrack::GetIterationNumber(void)
{
  return (iteration_number);
}

void CTrack::GetDimensions(float& a, float& b)
{
  if (best_hypothesis_is_a)
  {
    hypothesis_a.GetDimensions(a, b);
  }
  else
  {
    hypothesis_b.GetDimensions(a, b);
  }
}

void CTrack::SetKillFlag(void)
{
  kill_flag = true;
}

bool CTrack::GetKillFlag(void)
{
  return (kill_flag);
}

bool CTrack::GetFixedHypothesisFlag(void)
{
  return (flag_only_one_hypothesis_remaining);
}


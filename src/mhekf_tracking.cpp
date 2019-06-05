#include "mhekf_tracking.h"

CMhekf_Tracking::CMhekf_Tracking()
{
}

CMhekf_Tracking::~CMhekf_Tracking()
{
}

void CMhekf_Tracking::set(mhekf_tracker::KalmanConfiguration kalman_configuration,
                          mhekf_tracker::TrackManagementConfiguration track_management_configuration)
{
  kalman_configuration_ = kalman_configuration;
//  std::cout << "CMhekf_Tracking::set --> theta_sensor = " << kalman_configuration_.theta_sensor << std::endl;
  track_management_configuration_ = track_management_configuration;
}

void CMhekf_Tracking::updateTracks(std::vector<mhekf_tracker::DoubleHypothesisObservation> dh_observations,
                                   std::list<CTrack>& tracks, mhekf_tracker::EgoPose ego_pose)
{
  static int tracker_id = -1;
  float cov_x = 0.0;
  float cov_y = 0.0;

  int iter = -1;

  int secs = 0;
  int nano_secs = 0;

  bool hyp_a_match_obs_a = false;
  bool hyp_b_match_obs_b = false;

  bool associated = false;

  std::vector<int> tracks_succesfully_associated;
  std::vector<int> observations_succesfully_associated;

  const float IMPOSSIBLE_DISTANCE = 1000000.0;
  float mahalanobis_distance = IMPOSSIBLE_DISTANCE;
  float min_mahalanobis_distance = IMPOSSIBLE_DISTANCE;

  // Iterating over the whole observation vector

  //std::cout << ".....Starting association step ..................................................." << std::endl;

  for (std::vector<mhekf_tracker::DoubleHypothesisObservation>::iterator obs = dh_observations.begin();
      obs != dh_observations.end(); ++obs)
  {
    std::list<CTrack>::iterator coincidence = tracks.end();

    // This variable will store the minimum distance, so it is initialised
    // for each track with a value greater than any possible real value
    min_mahalanobis_distance = IMPOSSIBLE_DISTANCE;

    // Iterating over all the existing tracks
    for (std::list<CTrack>::iterator trk = tracks.begin(); trk != tracks.end(); ++trk)
    {
      mahalanobis_distance = trk->DoubleHypDoubleObsMahalanobisDistance(
          *obs, kalman_configuration_.mean_squared_error_noise_factor, ego_pose.x, ego_pose.y, ego_pose.theta,
          hyp_a_match_obs_a, hyp_b_match_obs_b);

      if (mahalanobis_distance < min_mahalanobis_distance)
      {
        min_mahalanobis_distance = mahalanobis_distance;
        coincidence = trk;
      }
    }

    if (coincidence != tracks.end())
    {
      //std::cout << "Mahalanobis distance = " << min_mahalanobis_distance << std::endl;

      // If the error in distance is lower than the threshold and the track has not been associated yet
      if (std::find(tracks_succesfully_associated.begin(), tracks_succesfully_associated.end(), coincidence->GetId())
          == tracks_succesfully_associated.end()
          && min_mahalanobis_distance < track_management_configuration_.mahalanobis_threshold)
      {
        associated = true;
        coincidence->Update(*obs, kalman_configuration_.mean_squared_error_noise_factor, ego_pose.x, ego_pose.y,
                            ego_pose.theta, associated);
        //std::cout << "Association success!! Track ID " << coincidence->GetId() << " --> Obs ID " << obs->id
        //    << std::endl;
        //std::cout << "Mahalanobis distance of associated observation = " << min_mahalanobis_distance << std::endl;

        tracks_succesfully_associated.push_back(coincidence->GetId());
        observations_succesfully_associated.push_back(obs->id);
      }
      else
      {
        //std::cout << "Discarding association" << std::endl;
      }
    }
  }

  //std::cout << "...................................................................Association Done!..." << std::endl
  //    << std::endl;

  //std::cout << "....Erasing old tracks..........................................................." << std::endl;
  // Erasing old tracks
  for (std::list<CTrack>::iterator trk = tracks.begin(); trk != tracks.end(); ++trk)
  {
    // If was not possible to associate the track, increment the counter of iterations without association
    if (std::find(tracks_succesfully_associated.begin(), tracks_succesfully_associated.end(), trk->GetId())
        == tracks_succesfully_associated.end())
    {
      trk->GetIterationNumber(iter);
      //std::cout << "Analyzing track with ID = " << trk->GetId() << "Num of iterations =" << iter << std::endl;
      // If there is no association but the track is fully initialized
      // we keep this track until it reaches the maximum position covariance
      if (iter > track_management_configuration_.filter_iterations_before_allowing_update_without_observation)
      {
        trk->UpdateWithoutObservation(secs, nano_secs);
        std::cout << "-------------------------Updating track without observation!!------------" << std::endl;
      }
      else
      {
        trk->SetKillFlag();
        //std::cout << "Setting Kill Flag of a not associated track!" << std::endl;
      }
    }

    trk->GetCovariance(cov_x, cov_y);
    if (cov_x > track_management_configuration_.max_covariance
        || cov_y > track_management_configuration_.max_covariance)
    {
      std::cout << "Setting Kill Flag for an excessive covariance track! Track ID = " << trk->GetId() << std::endl;
      std::cout << std::endl << "Covariances: cov_x = " << cov_x << "    cov_y = " << cov_y << std::endl;
      trk->SetKillFlag();
    }
  }

  //std::cout << std::endl << "Number of active tracks before erasing = " << tracks.size() << std::endl;

  std::list<CTrack>::iterator trk = tracks.begin();
  while (trk != tracks.end())
  {
    if (trk->GetKillFlag())
    {
      trk = tracks.erase(trk);
      //std::cout << "Track erased!" << std::endl;
    }
    else
    {
      ++trk;
    }
  }

  //std::cout << std::endl << "Number of active tracks after erasing = " << tracks.size() << std::endl;
  //std::cout << ".....................................................................Erasing Done!..." << std::endl
  //    << std::endl;

  // end of erasing

  //std::cout << std::endl << "...Adding new tracks........................................................."
  //    << std::endl;
  // Adding new tracks
  for (std::vector<mhekf_tracker::DoubleHypothesisObservation>::iterator obs = dh_observations.begin();
      obs != dh_observations.end(); ++obs)
  {
    if (std::find(observations_succesfully_associated.begin(), observations_succesfully_associated.end(), obs->id)
        == observations_succesfully_associated.end())
    {
      assert(
          obs->a.GetObservationOKforStartTrack() == obs->b.GetObservationOKforStartTrack()
              && "Error in CMhekf_Tracking::updateTracks hypothesis with different mixture in same observation!");
      if (obs->a.GetObservationOKforStartTrack() || obs->b.GetObservationOKforStartTrack())
      {
        CTrack new_track;
        new_track.SetStandardDeviations(kalman_configuration_);

        bool associated = true;
        new_track.Update(*obs, kalman_configuration_.mean_squared_error_noise_factor, ego_pose.x, ego_pose.y,
                         ego_pose.theta, associated);
        tracker_id++;
        new_track.SetId(tracker_id);
        tracks.push_back(new_track);
    //    std::cout << "New track successfully added!! Track id = " << new_track.GetId() << " --> Obs ID = " << obs->id
    //        << std::endl;
      }
      else
      {
      //  std::cout << "Unmatched observation discarded to start a track because no coincidence between CNN's"
      //      << std::endl << std::endl;
      }
    }
  }
  //std::cout << std::endl
  //    << ".................................................................New Tracks Addition Done!..." << std::endl;

  //std::cout << std::endl << "Exiting AssociationAndStartingOrErasingTracks function..." << std::endl;

  //Debug
//  std::cout << "Showing the likelihooh of all active tracks:" << std::endl;
//  for (std::list<CTrack>::iterator trk = tracks.begin(); trk != tracks.end(); ++trk)
//  {
//    std::cout << "TRK ID = " << trk->GetId() << "    Likelihood = " << trk->likelihood << std::endl;
//  }
}

void CMhekf_Tracking::updateTracks(std::vector<CObservation> sh_observations, std::list<CTrack>& tracks,
                                   mhekf_tracker::EgoPose ego_pose)
{
  static int tracker_id = -1;
  float cov_x = 0.0;
  float cov_y = 0.0;

  int iter = -1;

  int secs = 0;
  int nano_secs = 0;

  if(!sh_observations.empty())
  {
    sh_observations[0].GetTimestamp(secs, nano_secs);
  }

  bool hyp_a_match_obs_a = false;
  bool hyp_b_match_obs_b = false;

  bool associated = false;

  std::vector<int> tracks_succesfully_associated;
  std::vector<int> observations_succesfully_associated;

  const float IMPOSSIBLE_DISTANCE = 1000000.0;
  float mahalanobis_distance = IMPOSSIBLE_DISTANCE;
  float min_mahalanobis_distance = IMPOSSIBLE_DISTANCE;

  // Iterating over the whole observation vector

  std::cout << ".....Starting association step ..................................................." << std::endl;

  for (std::vector<CObservation>::iterator obs = sh_observations.begin(); obs != sh_observations.end();
      ++obs)
  {
    std::list<CTrack>::iterator coincidence = tracks.end();

    // This variable will store the minimum distance, so it is initialised
    // for each track with a value greater than any possible real value
    min_mahalanobis_distance = IMPOSSIBLE_DISTANCE;

    // Iterating over all the existing tracks
    for (std::list<CTrack>::iterator trk = tracks.begin(); trk != tracks.end(); ++trk)
    {
      mahalanobis_distance = trk->MahalanobisDistance(*obs, kalman_configuration_.mean_squared_error_noise_factor,
                                                      ego_pose.x, ego_pose.y, ego_pose.theta);

      if (mahalanobis_distance < min_mahalanobis_distance)
      {
        min_mahalanobis_distance = mahalanobis_distance;
        coincidence = trk;
      }
    }

    if (coincidence != tracks.end())
    {
      std::cout << "Mahalanobis distance = " << min_mahalanobis_distance << std::endl;

      // If the error in distance is lower than the threshold and the track has not been associated yet
      if (std::find(tracks_succesfully_associated.begin(), tracks_succesfully_associated.end(), coincidence->GetId())
          == tracks_succesfully_associated.end()
          && min_mahalanobis_distance < track_management_configuration_.mahalanobis_threshold)
      {
        std::cout << "Association success!! Track ID " << coincidence->GetId() << " --> Obs ID " << obs->GetId()
            << std::endl;
        std::cout << "Mahalanobis distance of associated observation = " << min_mahalanobis_distance << std::endl;

        associated = true;
        coincidence->Update(*obs, kalman_configuration_.mean_squared_error_noise_factor, ego_pose.x, ego_pose.y,
                            ego_pose.theta, associated);


        tracks_succesfully_associated.push_back(coincidence->GetId());
        observations_succesfully_associated.push_back(obs->GetId());
      }
      else
      {
        std::cout << "Discarding association" << std::endl;
      }
    }
  }

  std::cout << "...................................................................Association Done!..." << std::endl
      << std::endl;

  std::cout << "....Erasing old tracks..........................................................." << std::endl;
  // Erasing old tracks
  for (std::list<CTrack>::iterator trk = tracks.begin(); trk != tracks.end(); ++trk)
  {
    std::cout << "Analyzing track with ID = " << trk->GetId() << std::endl;
    if (std::find(tracks_succesfully_associated.begin(), tracks_succesfully_associated.end(), trk->GetId())
        == tracks_succesfully_associated.end())
    {
      std::cout << "Track not associated!" << std::endl;
      trk->GetIterationNumber(iter);
      std::cout << "Num of iterations =" << iter << std::endl;

      // If there is no association but the track is fully initialized
      // we keep this track until it reaches the maximum position covariance
      if (iter > track_management_configuration_.filter_iterations_before_allowing_update_without_observation)
      {
        std::cout << "-------------------------Updating track without observation!!------------" << std::endl;
        trk->UpdateWithoutObservation(secs, nano_secs);
      }
      else
      {
        std::cout << "Setting Kill Flag of an uninitialised and not associated track!" << std::endl;
        trk->SetKillFlag();
      }
    }
    else
    {
      std::cout << "it is an associated track!" << std::endl;
    }

    trk->GetCovariance(cov_x, cov_y);
    std::cout << std::endl << "Covariances: cov_x = " << cov_x << "    cov_y = " << cov_y << std::endl;
    if (cov_x > track_management_configuration_.max_covariance
        || cov_y > track_management_configuration_.max_covariance)
    {
      std::cout << "Setting Kill Flag for an excessive covariance track!" << std::endl;
      trk->SetKillFlag();
    }
    float a = 0.0;
    float b = 0.0;
    trk->GetDimensions(a,b);
    std::cout << std::endl << "Dimensions: a = " << a << "    b = " << b << std::endl;
    if(a > 2.2 && b > 2.2)
    {
      std::cout << "Setting Kill Flag for a too big track!" << std::endl;
      trk->SetKillFlag();
    }
    if(a > 5.0 || b > 5.0)
    {
      std::cout << "Setting Kill Flag for a too big track!" << std::endl;
      trk->SetKillFlag();
    }
  }

  std::cout << std::endl << "Number of active tracks before erasing = " << tracks.size() << std::endl;

  std::list<CTrack>::iterator trk = tracks.begin();
  while (trk != tracks.end())
  {
    if (trk->GetKillFlag())
    {
      std::cout << "Track " << trk->GetId() << " erased!" << std::endl;
      trk = tracks.erase(trk);
    }
    else
    {
      ++trk;
    }
  }

  std::cout << std::endl << "Number of active tracks after erasing = " << tracks.size() << std::endl;
  std::cout << ".....................................................................Erasing Done!..." << std::endl
      << std::endl;

  // end of erasing

  std::cout << std::endl << "...Adding new tracks........................................................."
      << std::endl;
  // Adding new tracks
  for (std::vector<CObservation>::iterator obs = sh_observations.begin();
      obs != sh_observations.end(); ++obs)
  {
    if (std::find(observations_succesfully_associated.begin(), observations_succesfully_associated.end(), obs->GetId())
        == observations_succesfully_associated.end())
    {
      if (obs->GetObservationOKforStartTrack())
      {
        CTrack new_track;
        new_track.SetStandardDeviations(kalman_configuration_);

        std::cout << "New track successfully added!! Track id = " << new_track.GetId() << " --> Obs ID = " << obs->GetId()
            << std::endl;

        bool associated = true;
        new_track.Update(*obs, kalman_configuration_.mean_squared_error_noise_factor, ego_pose.x, ego_pose.y,
                         ego_pose.theta, associated);
        tracker_id++;
        new_track.SetId(tracker_id);
        tracks.push_back(new_track);
      }
      else
      {
        std::cout << "Unmatched observation discarded to start a track because no coincidence between CNN's"
            << std::endl << std::endl;
      }
    }
  }
//  std::cout << std::endl
//      << ".................................................................New Tracks Addition Done!..." << std::endl;
//
//  std::cout << std::endl << "Exiting AssociationAndStartingOrErasingTracks function..." << std::endl;

  //Debug
//  std::cout << "Showing the likelihooh of all active tracks:" << std::endl;
//  for (std::list<CTrack>::iterator trk = tracks.begin(); trk != tracks.end(); ++trk)
//  {
//    std::cout << "TRK ID = " << trk->GetId() << "    Likelihood = " << trk->likelihood << std::endl;
//  }
}


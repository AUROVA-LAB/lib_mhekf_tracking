#ifndef _MHEKF_TRACKING_H
#define _MHEKF_TRACKING_H

#include "observation.h"
#include "track.h"

#include <list>

class CMhekf_Tracking
{
private:
  mhekf_tracker::KalmanConfiguration kalman_configuration_;
  mhekf_tracker::TrackManagementConfiguration track_management_configuration_;
public:

  CMhekf_Tracking();
  ~CMhekf_Tracking();

  void set(mhekf_tracker::KalmanConfiguration kalman_configuration,
           mhekf_tracker::TrackManagementConfiguration track_management_configuration);

  void updateTracks(std::vector<mhekf_tracker::DoubleHypothesisObservation> dh_observations, std::list<CTrack>& tracks,
                    mhekf_tracker::EgoPose ego_pose);

  void updateTracks(std::vector<CObservation> sh_observations, std::list<CTrack>& tracks,
                    mhekf_tracker::EgoPose ego_pose);
};

#endif

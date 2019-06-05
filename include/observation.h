/*
 * observation.h
 *
 *  Created on: Dec 7, 2016
 *      Author: idelpino
 */

#ifndef INCLUDE_OBSERVATION_H_
#define INCLUDE_OBSERVATION_H_

class CObservation
{
  private:

    float x;
    float y;
    float z;
    float theta;
    float mean_squared_error;
    float objectness;
    float probability;
    bool  ok_for_start_track;

    float length_side_a;
    float length_side_b;
    float height;

    int timestamp_seconds;
    int timestamp_nano_seconds;

    int id;

    bool initialized;

  public:

    CObservation ();

    void Set ( float x_obs, float y_obs, float theta_obs, float mean_sq_error, float object_confidence, float length_side_a_obs,
	       float length_side_b_obs, int seconds, int nano_seconds, int obs_id, float prob );

    void Set ( float x_obs, float y_obs, float z_obs, float theta_obs, float mean_sq_error, float object_confidence, float length_side_a_obs,
	       float length_side_b_obs, float height_obs, int seconds, int nano_seconds, int obs_id, float prob );

    void Set ( float x_obs, float y_obs, float theta_obs, float mean_sq_error, float length_side_a_obs,
	       float length_side_b_obs, int seconds, int nano_seconds, int obs_id, float prob );

    void Get ( float& x_obs, float& y_obs, float& z_obs, float& theta_obs, float& mean_sq_error, float& object_confidence, float& length_side_a_obs,
	       float& length_side_b_obs, float& height_obs, int& seconds, int& nano_seconds, int& obs_id, float& prob );

    void Get ( float& x_obs, float& y_obs, float& theta_obs, float& mean_sq_error, float& length_side_a_obs,
	       float& length_side_b_obs, int& seconds, int& nano_seconds, int& obs_id, float& prob );

    void ObservationOKforStartTrack( void );

    bool GetObservationOKforStartTrack (void);

    void GetTimestamp ( int& secs, int& nano_secs);

    void GetId ( int& obs_id );
    int GetId ( void );

    float GetError ( void );
    float GetObjectness ( void );

    float GetLengthA ( void );
    float GetLengthB ( void );

  virtual
    ~CObservation ();

  float getProbability() const
  {
    return probability;
  }

  void setProbability(float probability)
  {
    this->probability = probability;
  }

  float getHeight() const
  {
    return height;
  }

  float getZ() const
  {
    return z;
  }
};

#endif /* INCLUDE_OBSERVATION_H_ */

/*
 * Observation.cpp
 *
 *  Created on: Dec 7, 2016
 *      Author: idelpino
 */

#include "observation.h"

#include <iostream>

CObservation::CObservation()
{
  x = 0.0;
  y = 0.0;
  z = -0.75;
  theta = 0.0;

  mean_squared_error = 0.0;
  objectness = 0.0;

  probability = 0.0;

  length_side_a = 0.0;
  length_side_b = 0.0;
  height = 1.5;

  timestamp_seconds = 0;
  timestamp_nano_seconds = 0;

  id = 0;

  initialized = false;
  ok_for_start_track = false;
}

CObservation::~CObservation()
{

}

void CObservation::Set(float x_obs, float y_obs, float theta_obs, float mean_sq_error, float object_confidence,
                       float length_side_a_obs, float length_side_b_obs, int seconds, int nano_seconds, int obs_id, float prob)
{
  x = x_obs;
  y = y_obs;
  theta = theta_obs;

  mean_squared_error = mean_sq_error;
  objectness = object_confidence;

  probability = prob;

  length_side_a = length_side_a_obs;
  length_side_b = length_side_b_obs;

  timestamp_seconds = seconds;
  timestamp_nano_seconds = nano_seconds;

  id = obs_id;

  initialized = true;

}

void CObservation::Set(float x_obs, float y_obs, float z_obs, float theta_obs, float mean_sq_error,
                       float object_confidence, float length_side_a_obs, float length_side_b_obs, float height_obs,
                       int seconds, int nano_seconds, int obs_id, float prob)
{
  x = x_obs;
  y = y_obs;
  z = z_obs;
  theta = theta_obs;

  mean_squared_error = mean_sq_error;
  objectness = object_confidence;

  probability = prob;

  length_side_a = length_side_a_obs;
  length_side_b = length_side_b_obs;
  height = height_obs;

  timestamp_seconds = seconds;
  timestamp_nano_seconds = nano_seconds;

  id = obs_id;

  initialized = true;
}

void CObservation::Set(float x_obs, float y_obs, float theta_obs, float mean_sq_error, float length_side_a_obs,
                       float length_side_b_obs, int seconds, int nano_seconds, int obs_id, float prob)
{
  x = x_obs;
  y = y_obs;
  theta = theta_obs;

  mean_squared_error = mean_sq_error;

  probability = prob;

  length_side_a = length_side_a_obs;
  length_side_b = length_side_b_obs;

  timestamp_seconds = seconds;
  timestamp_nano_seconds = nano_seconds;

  id = obs_id;

  initialized = true;
}

void CObservation::Get(float& x_obs, float& y_obs, float& z_obs, float& theta_obs, float& mean_sq_error,
                       float& object_confidence, float& length_side_a_obs, float& length_side_b_obs, float& height_obs,
                       int& seconds, int& nano_seconds, int& obs_id, float& prob)
{
  if (initialized)
  {
    x_obs = x;
    y_obs = y;
    z_obs = z;

    theta_obs = theta;

    mean_sq_error = mean_squared_error;
    object_confidence = objectness;

    prob = probability;

    length_side_a_obs = length_side_a;
    length_side_b_obs = length_side_b;

    height_obs = height;

    seconds = timestamp_seconds;
    nano_seconds = timestamp_nano_seconds;

    obs_id = id;

  }
  else
  {
    std::cout << "Warning!! Trying to use an uninitialized observation" << std::endl;
  }
}

void CObservation::Get(float& x_obs, float& y_obs, float& theta_obs, float& mean_sq_error, float& length_side_a_obs,
                       float& length_side_b_obs, int& seconds, int& nano_seconds, int& obs_id, float& prob)
{
  if (initialized)
  {
    x_obs = x;
    y_obs = y;
    theta_obs = theta;

    mean_sq_error = mean_squared_error;

    length_side_a_obs = length_side_a;
    length_side_b_obs = length_side_b;

    prob = probability;

    seconds = timestamp_seconds;
    nano_seconds = timestamp_nano_seconds;

    obs_id = id;

  }
  else
  {
    std::cout << "Warning!! Trying to use an uninitialized observation" << std::endl;
  }
}

void CObservation::GetTimestamp(int& secs, int& nano_secs)
{
  secs = timestamp_seconds;
  nano_secs = timestamp_nano_seconds;
}

void CObservation::ObservationOKforStartTrack(void)
{
  ok_for_start_track = true;
}

bool CObservation::GetObservationOKforStartTrack(void)
{
  return (ok_for_start_track);
}

float CObservation::GetError(void)
{
  return (mean_squared_error);
}

float CObservation::GetObjectness(void)
{
  return (objectness);
}

void CObservation::GetId(int& obs_id)
{
  obs_id = id;
}

int CObservation::GetId()
{
  return (id);
}

float CObservation::GetLengthA(void)
{
  return (length_side_a);
}

float CObservation::GetLengthB(void)
{
  return (length_side_b);
}

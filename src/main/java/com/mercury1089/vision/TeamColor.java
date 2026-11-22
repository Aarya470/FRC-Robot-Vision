package com.mercury1089.vision;

/** Which alliance a robot is on, if the camera can tell. */
public enum TeamColor {
  RED,
  BLUE,

  /** The camera can't tell, or it is not a robot */
  UNKNOWN
}

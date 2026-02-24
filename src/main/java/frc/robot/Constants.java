// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
  public static final int CAN_motor_intake_roller = 30;
  public static final int CAN_motor_intake_updown = 31;
  public static final int CAN_motor_intake_conveyor = 32;
  public static final int CAN_motor_shooter_left = 35;
  public static final int CAN_motor_shooter_right = 36;
  public static final int CAN_motor_kicker = 37;
  public static final int CAN_motor_angle = 38;
  public static final int CAN_encoder_angle = 39;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class VisionConstants
  {
    public static final String LIMELIGHT_NAME = "limelight-riveter";

    public static final double waitTime = 1;
    public static final double validationTime = 0.3;

    public static final double TOLERANCE = 0.01;
  }
  public static final class Shooter
  {

    //public static final double shootSpeed = 1800;   // TOWER FRONT lowest angle
    //public static final double shootSpeed = 2000;   // TOWER SIDE (back to wall) lowest angle
    //public static final double shootSpeed = 1850;   // TRENCH lowest angle
    //public static final double shootSpeed = 2100;   // CORNER lowest angle
    public static final double shootSpeed = 1600;   // POINT BLANK 27" bumper to HUB highest angle
  }

}

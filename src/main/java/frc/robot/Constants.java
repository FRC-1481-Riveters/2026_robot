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

  public static final int CAN_motor_intake_roller_inner = 30;
  public static final int CAN_motor_intake_updown = 31;
  public static final int CAN_motor_intake_conveyor = 32;
  public static final int CAN_motor_intake_roller_outer = 33;

  public static final int CAN_motor_shooter_left = 35;
  public static final int CAN_motor_shooter_right = 36;
  public static final int CAN_motor_kicker = 37;
  public static final int CAN_motor_angle = 38;
  public static final int CAN_encoder_angle = 39;

  public static final double loopPeriodWatchdogSecs = 0.2;

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
    public static final double shootSpeedPointBlank = 1800; //1600; // 30" from HUB highest angle
    public static final double shootSpeedTowerFront = 2000; //1780; // TOWER FRONT lowest angle
    public static final double shootSpeedTrench = 2050;     // TRENCH lowest angle
    public static final double shootSpeedTowerBack = 2300;
    public static final double shootSpeedCorner = 2530;     // CORNER lowest angle

    //-8.5
    //-25.4
    public static final double shooterAnglePositionMax = -8.6;   // lowest possible shooting angle (longest shot)
    public static final double shooterAnglePositionTowerBack = -9.0; //-2.5;
    public static final double shooterAnglePositionTower = -15.5; //-2.5;
    public static final double shooterAnglePositionMin = -24.5;  // highest possible shooting angle (shortest shot)

    public static final double kickPercent = 0.75;
    public static final double shootSpeed = 1600;
    public static final double kickerSpeed = 2500;
    public static final double conveyorSpeed = 0.3;
  }

  public static final class Intake
  {
    public static final double upDownPositionDown = -27.6;
    public static final double upDownPosition30Degrees = -21.0;
     public static final double upDownPosition10Degrees = -24.0;
    public static final double upDownPositionUp = 0.0;
    public static final double rollersPercentMax = 0.55;
  }

  public static final class Drive
  {
    public static final double bumpSpeed = 0.4; //0.3 was kind, 0.5 too much air
    public static final double pickupSpeed = 0.20; 
  }
}

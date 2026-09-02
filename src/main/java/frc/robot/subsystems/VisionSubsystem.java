package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.LimelightHelpers.*;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class VisionSubsystem extends SubsystemBase {
  private RawFiducial[] fiducials;
  private CommandSwerveDrivetrain m_commandSwerveDrivetrain;
  private AprilTagFieldLayout tagLayout;
  private final Field2d m_fieldBack = new Field2d();
  private final Field2d m_fieldLeft = new Field2d();
  private final Field2d m_fieldRight = new Field2d();
  private Pose2d poseBack, poseLeft, poseRight;
  private Timer shooterCamAge = new Timer();


  public VisionSubsystem(CommandSwerveDrivetrain commandSwerveDrivetrain) {
    m_commandSwerveDrivetrain = commandSwerveDrivetrain;
    config();
    tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);    

    var driveState = m_commandSwerveDrivetrain.getState();

    double headingDegrees = driveState.Pose.getRotation().getDegrees();
    LimelightHelpers.SetRobotOrientation("limelight-back", headingDegrees, 0, 0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation("limelight-left", headingDegrees, 0, 0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation("limelight-right", headingDegrees, 0, 0, 0, 0, 0);

    // Do this in either robot or subsystem init
    SmartDashboard.putData("FieldBack", m_fieldBack);
    SmartDashboard.putData("FieldLeft", m_fieldLeft);
    SmartDashboard.putData("FieldRight", m_fieldRight);
  }

  public static class NoSuchTargetException extends RuntimeException {
    public NoSuchTargetException(String message) {
      super(message);
    }
  }

  public void config() {

    /* 
    LimelightHelpers.setCameraPose_RobotSpace(
        "limelight-back",
        -0.305,
        0 ,
        0.606,
        0,
        0 ,
        180
        );
    LimelightHelpers.setCameraPose_RobotSpace(
        "limelight-left",
        -0.203,
        -0.381,
        0.673,
        0,
        0,
        90
        );
    LimelightHelpers.setCameraPose_RobotSpace(
        "limelight-right",
        -0.203,
        0.381,
        0.673,
        0,
        0,
        -90
        );
      */

        /*
        Use the Limelight's internal IMU in addition to the swerve subsystem's Pigeon IMU
        see:
        https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization-megatag2#using-the-internal-imu-with-megatag2
        */
        // DOES NOT WORK GOOD setting this to 4
        LimelightHelpers.SetIMUMode("limelight-back", 0 );
        LimelightHelpers.SetIMUMode("limelight-left", 0 );
        LimelightHelpers.SetIMUMode("limelight-right", 0 );

        Logger.recordOutput("Vision/shooterCamAge", 0.0 );
        shooterCamAge.start();
    }
      
    public void LimelightPipelines()
    { 
          
      LimelightHelpers.setPipelineIndex("limelight-back",2);
        LimelightHelpers.setPipelineIndex("limelight-left",2);
        LimelightHelpers.setPipelineIndex("limelight-right",2);          
    }
    


  @Override
  public void periodic() {
      fiducials = LimelightHelpers.getRawFiducials("limelight-back");

      var driveState = m_commandSwerveDrivetrain.getState();
      double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

      double headingDegrees = driveState.Pose.getRotation().getDegrees();
      LimelightHelpers.SetRobotOrientation("limelight-back", headingDegrees, 0, 0, 0, 0, 0);
      LimelightHelpers.SetRobotOrientation("limelight-left", headingDegrees, 0, 0, 0, 0, 0);
      LimelightHelpers.SetRobotOrientation("limelight-right", headingDegrees, 0, 0, 0, 0, 0);

      if(Math.abs(omegaRps) > 2.0) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
        return;

      /* Limelight tips:
      -- Start with SENSOR GAIN = 15
      -- Set FLICKER CORRECTION to 60 Hz
      -- reduce exposure until tags start flickering
      -- set DETECTOR DOWNSCALE to 2
      -- set QUALITY THRESHOLD to 2
      -- enable FULL 3D TARGETING
      */

      // Ignore side Limelights if we are going over the BUMP
      double roll = m_commandSwerveDrivetrain.getPigeon2().getRoll().getValueAsDouble();
      double pitch = m_commandSwerveDrivetrain.getPigeon2().getPitch().getValueAsDouble();

      if( Math.abs(roll) < 3.0 && Math.abs(pitch) < 3.0 )
      {
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-back");
        if( mt2 != null )
        {
          if(mt2.tagCount >= 2)
          {
            poseBack = mt2.pose;
            m_commandSwerveDrivetrain.updateOdometry(poseBack, true, mt2.timestampSeconds);

            // just a hack for visualization - seems like Field2d is bugged, because Glass shows degrees instead of radians
            Rotation2d hack = new Rotation2d(poseBack.getRotation().getRadians() / (360.0 / (2.0 * Math.PI)));            
            m_fieldBack.setRobotPose(poseBack.getX(), poseBack.getY(), hack);
            shooterCamAge.reset();
          }
        }

        LimelightHelpers.PoseEstimate mtCamLeft = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-left");
        if( mtCamLeft != null )
        {
          if(mtCamLeft.tagCount >= 2)
          {            
            poseLeft = mtCamLeft.pose;
            m_commandSwerveDrivetrain.updateOdometry(poseLeft, true, mtCamLeft.timestampSeconds);

            // just a hack for visualization - seems like Field2d is bugged, because Glass shows degrees instead of radians
            Rotation2d hack = new Rotation2d(poseLeft.getRotation().getRadians() / (360.0 / (2.0 * Math.PI)));            
            m_fieldLeft.setRobotPose(poseLeft.getX(), poseLeft.getY(), hack);  
          }
        }

        LimelightHelpers.PoseEstimate mtCamRight = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-right");
        if( mtCamRight != null )
        {
          if(mtCamRight.tagCount >= 2)
          {
           poseRight = mtCamRight.pose;
            m_commandSwerveDrivetrain.updateOdometry(poseRight, true, mtCamRight.timestampSeconds);

            // just a hack for visualization - seems like Field2d is bugged, because Glass shows degrees instead of radians
            Rotation2d hack = new Rotation2d(poseRight.getRotation().getRadians() / (360.0 / (2.0 * Math.PI)));            
            m_fieldRight.setRobotPose(poseRight.getX(), poseRight.getY(), hack);
          }
        }
      }

      Logger.recordOutput("Vision/shooterCamAge", shooterCamAge.get() );

  }

  public boolean isInsideField(CommandSwerveDrivetrain commandSwerveDrivetrain) {
    m_commandSwerveDrivetrain = commandSwerveDrivetrain;
    var driveState = m_commandSwerveDrivetrain.getState();
    // Directions are relative to driver's perspective
    // Blue Left: y = 0.7587x + 267.2
    // Blue Right: y = -0.7587x - 49.95
    // Red Left: y = 0.7587x - 474.187
    // Red Right: y = -0.7587x + 791.337

    double slope = Units.inchesToMeters(0.7587);
    double x = driveState.Pose.getX();
    double y = driveState.Pose.getY();

    boolean isInsideBlueLeft = (y < slope*x + Units.inchesToMeters(267.2));
    boolean isInsideBlueRight = (y > -slope*x + Units.inchesToMeters(49.95));
    boolean isInsideRedLeft = (y > slope*x - Units.inchesToMeters(474.187));
    boolean isInsideRedRight = (y < -slope*x + Units.inchesToMeters(791.337));
    boolean isInsideRectangle = (x > 0 && x < tagLayout.getFieldLength() && y > 0 && y < tagLayout.getFieldWidth());

    if(isInsideBlueRight && isInsideBlueLeft && isInsideRedLeft && isInsideRedRight && isInsideRectangle) {
      return true;
    } else {
      return false;
    }
  }

  public void limelightSlow( boolean slow )
  {
  
  //change before practice matches (after config)
    if( slow )
    {
      LimelightHelpers.SetThrottle("limelight-back", 100);
      LimelightHelpers.SetThrottle("limelight-left", 100);
      LimelightHelpers.SetThrottle("limelight-right", 100);
    }
    else
    {
      LimelightHelpers.SetThrottle("limelight-back", 0);
      LimelightHelpers.SetThrottle("limelight-left", 0);
      LimelightHelpers.SetThrottle("limelight-right", 0);
    }
  }

  public RawFiducial getClosestFiducial() {
    if (fiducials == null || fiducials.length == 0) {
        throw new NoSuchTargetException("No fiducials found.");
    }

    RawFiducial closest = fiducials[0];
    double minDistance = closest.ta;

    for (RawFiducial fiducial : fiducials) {
        if (fiducial.ta > minDistance) {
            closest = fiducial;
            minDistance = fiducial.ta;
        }
    }

    return closest;
  }

  public RawFiducial getFiducialWithId(int id) {
  
    for (RawFiducial fiducial : fiducials) {
        if (fiducial.id == id) {
            return fiducial;
        }
    }
    throw new NoSuchTargetException("Can't find ID: " + id);
  }

public RawFiducial getFiducialWithId(int id, boolean verbose) {
  StringBuilder availableIds = new StringBuilder();

  for (RawFiducial fiducial : fiducials) {
      if (availableIds.length() > 0) {
          availableIds.append(", ");
      } //Error reporting
      availableIds.append(fiducial.id);
      
      if (fiducial.id == id) {
          return fiducial;
      }
  }
  throw new NoSuchTargetException("Cannot find: " + id + ". IN view:: " + availableIds.toString());
  }

  public double getTX(){
    return LimelightHelpers.getTX(VisionConstants.LIMELIGHT_NAME);
  }
  public double getTY(){
    return LimelightHelpers.getTY(VisionConstants.LIMELIGHT_NAME);
  }
  public double getTA(){
    return LimelightHelpers.getTA(VisionConstants.LIMELIGHT_NAME);
  }
  public boolean getTV(){
    return LimelightHelpers.getTV(VisionConstants.LIMELIGHT_NAME);
  }

  public double getClosestTX(){
    return getClosestFiducial().txnc;
  }
  public double getClosestTY(){
    return getClosestFiducial().tync;
  }
  public double getClosestTA(){
    return getClosestFiducial().ta;
  }

  private boolean left17, right17, back17;

  public void testClear()
  {
      System.out.println("Test Vision: hold tag at least 4 feet away from left, right, and back Limelights");
      left17 = false;
      right17 = false;
      back17 = false;
  }

  public boolean all17()
  {
    if( LimelightHelpers.getTA("limelight-back") > 0.1 && (back17 == false))
    {
      back17 = true;
      System.out.println("Back camera: AprilTag detected");
    }
    if( LimelightHelpers.getTA("limelight-left") > 0.1 && (left17 == false))
    {
      left17 = true;
      System.out.println("Left camera: AprilTag detected");
    }
    if( LimelightHelpers.getTA("limelight-right") > 0.1 && (right17 == false))
    {
      right17 = true;
      System.out.println("Right camera: AprilTag detected");
    }

    return( back17 && left17 && right17 );
  }
}

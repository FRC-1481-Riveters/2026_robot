// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.PointWheelsAt;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import frc.robot.subsystems.VisionSubsystem;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;





public class RobotContainer {
    private final SendableChooser<Command> autoChooser;

    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); 
    private double shootingSpeed = 1800;
    Translation2d hubPosition;
    
    private final Telemetry logger = new Telemetry(MaxSpeed);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final VisionSubsystem m_Vision = new VisionSubsystem(drivetrain);
    private final Intake m_Intake = new Intake();
    private final Shooter m_Shooter = new Shooter();
    private LoggedNetworkNumber shooterSpeed = new LoggedNetworkNumber("/Tuning/ShooterSpeed", Constants.Shooter.shootSpeed);
    private LoggedNetworkNumber kickerSpeed = new LoggedNetworkNumber("/Tuning/KickerSpeed", Constants.Shooter.kickerSpeed);
    private LoggedNetworkNumber conveyorSpeed = new LoggedNetworkNumber("/Tuning/ConveyorSpeed", Constants.Shooter.conveyorSpeed);

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController operatorJoystick = new CommandXboxController(1);

    private boolean bumpSpeedPressedOperator = false;
    private boolean bumpSpeedPressedDriver = false;
    private boolean pickupSpeedPressedOperator = false;
    private boolean pickupSpeedPressedDriver = false;
    private boolean bAutoAimDone = false;


    public RobotContainer() {
        setupNamedCommands();
        configureBindings();
        for (int port = 5801; port <= 5809; port++) {
            PortForwarder.add(port, "limelight-back.local", port);
            PortForwarder.add(port, "10.14.81.11", port);
            PortForwarder.add(port, "limelight-left.local", port);
            PortForwarder.add(port, "10.14.81.12", port);
            PortForwarder.add(port, "limelight-right.local", port);
            PortForwarder.add(port, "10.14.81.13", port);
        }

        autoChooser = AutoBuilder.buildAutoChooser("Nothing");
        SmartDashboard.putData("Auto Mode", autoChooser);

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private void setupNamedCommands()
    {
        NamedCommands.registerCommand("cmdWait4", Commands.waitSeconds(4.0));
        NamedCommands.registerCommand("IntakeLower", IntakeLower());
        NamedCommands.registerCommand("RollerStop", RollerStop());
        NamedCommands.registerCommand("ShootShortSpinup", ShootShortSpinup());
        NamedCommands.registerCommand("Shoot", Shoot());
        NamedCommands.registerCommand("AutoAim", AutoAim());
        NamedCommands.registerCommand("AutoShooter", Commands.runOnce( ()->autoShooterRPM()) );
    }

    private Command IntakeLower()
    {
        return 
            Commands.runOnce( ()->m_Intake.setUpDownPosition(Constants.Intake.upDownPositionDown), m_Intake )
                .andThen( Commands.waitSeconds(0.5))
                .andThen( Commands.runOnce( ()->m_Intake.setRollerPercentOutput(-Constants.Intake.rollersPercentMax * 0.5) ) )
                .andThen( Commands.waitSeconds(1.5))
                .andThen( Commands.runOnce( ()->m_Intake.setRollerPercentOutput(-Constants.Intake.rollersPercentMax) ) )
                .andThen( Commands.waitSeconds(0.5));
    }

    private Command RollerStop()
    {
        return 
            Commands.runOnce( ()->m_Intake.setRollerPercentOutput(0) );
    }

    private Command ShootShortSpinup()
    {
        return Commands.runOnce( ()->this.setShooter( Constants.Shooter.shootSpeedPointBlank, Constants.Shooter.shooterAnglePositionMin ));
        }

    private Command Shoot()
    {
        return Commands.runOnce( ()->m_Shooter.setShooterRPM(shootingSpeed) ) 
        .andThen(Commands.waitUntil( m_Shooter::isVelocityWithinTolerance) )
        .andThen(Commands.runOnce( ()->m_Shooter.setKickerRPM(kickerSpeed.get())) )
        .andThen(Commands.runOnce( ()->m_Intake.setConveyorPercentOutput(conveyorSpeed.get())) )
        .andThen(Commands.waitSeconds(1.0))
        .andThen(Commands.runOnce( ()->m_Intake.setRollerPercentOutput(-Constants.Intake.rollersPercentMax)))
        .andThen(Commands.waitSeconds(3.5))
        .andThen(Commands.runOnce( ()->m_Intake.setUpDownPosition( Constants.Intake.upDownPosition30Degrees ) ))
        .andThen(Commands.waitSeconds(10.0))
        .andThen(ShooterStop());
    }

    private Command AutoAim()
    {
        return drivetrain.applyRequest(() ->
            drive.withVelocityX( -deadBandLeftY() * MaxSpeed ) // Drive forward/backward
                .withVelocityY( -deadBandLeftX() * MaxSpeed )  // Drive left/right
                .withRotationalRate( AutoAimCalculate() ) // Positive = counterclockwise
        )
        .until( this::AutoAimDone );
    }

    private Command ShooterStop()
    {
        return Commands.runOnce( ()->m_Intake.setConveyorPercentOutput(0))
        .andThen(Commands.runOnce( ()->m_Intake.setRollerPercentOutput(0)))
        .andThen(Commands.runOnce( ()->AutoAimClear() ) )
        .andThen(Commands.waitSeconds(0.25))
        .andThen(Commands.runOnce( ()->m_Shooter.setKickerRPM(0)) )
        .andThen(Commands.waitSeconds(0.25))
        .andThen(Commands.runOnce( ()->m_Shooter.setShooterRPM(0) ) );
    }

    private double deadBandLeftX() {
        double newValue = joystick.getLeftX();
        if ((newValue <= 0.10) && (newValue >= -0.10))
        {
            return 0.0;
        }
        else if (newValue < 0) 
        {
            return (-1*(newValue*newValue));
        }
        else
        {
            return newValue*newValue;
        }

    }

    private double deadBandRightX() {
        double newValue = joystick.getRightX();
        if ((newValue <= 0.08) && (newValue >= -0.08))
        {
            return 0.0;
        }
        else if (newValue < 0) 
        {
            return (-1*(newValue*newValue));
        }
        else
        {
            return newValue*newValue;
        }

    }

    private void autoShooterRPM()
    {
        Pose2d robotPose = drivetrain.getState().Pose;
        double distance = robotPose.getTranslation().getDistance(hubPosition);
        double angle;
        double speed;
        double percent=0;

        double distance_30inch = 1.76;      // distance between robot center and hub with 30 inches between bumpers and hub base
        double distance_tower_front = 2.87; // distance to hub when back of the robot is at the front of the tower
        double distance_trench = 3.56;      // distance to hub when back of the robot is next to trench, back against the wall
        double distance_tower_back = 3.94;  // distance to hub when back of the robot is at the alliance wall next to the tower
        double distance_corner = 5.25;      // distance to hub when back of the robot is in either alliance corner

        /*
        Interpolate between these positions the hard way
        - a real way would be to build an array of distance+angle+position and interpolate between
        - but here we'll just do it the hard way with if-thens so it's easy to understand
        */
        if( distance < distance_30inch )
        {
            angle = Constants.Shooter.shooterAnglePositionMin;
            speed = Constants.Shooter.shootSpeedPointBlank;
        }
        else if( distance < distance_tower_front )
        {
            percent = (distance - distance_30inch) / (distance_tower_front - distance_30inch);
            angle = 
                Constants.Shooter.shooterAnglePositionMin + 
                (percent * (Constants.Shooter.shooterAnglePositionTower - Constants.Shooter.shooterAnglePositionMin));
            speed =
                Constants.Shooter.shootSpeedPointBlank + 
                (percent * (Constants.Shooter.shootSpeedTowerFront - Constants.Shooter.shootSpeedPointBlank));
        }
        else if( distance < distance_trench )
        {
            percent = (distance - distance_tower_front) / (distance_trench - distance_tower_front);
            angle = Constants.Shooter.shooterAnglePositionTower;
            speed =
                Constants.Shooter.shootSpeedTowerFront + 
                (percent * (Constants.Shooter.shootSpeedTrench - Constants.Shooter.shootSpeedTowerFront));
        }
        else if( distance < distance_tower_back )
        {
            percent = (distance - distance_trench) / (distance_tower_back - distance_trench);
            angle = Constants.Shooter.shooterAnglePositionTower;
            speed =
                Constants.Shooter.shootSpeedTrench + 
                (percent * (Constants.Shooter.shootSpeedTowerBack - Constants.Shooter.shootSpeedTrench));
        }
        else if( distance < distance_corner )
        {
            percent = (distance - distance_tower_back) / (distance_corner - distance_tower_back);
            angle = 
                Constants.Shooter.shooterAnglePositionTower + 
                (percent * (Constants.Shooter.shooterAnglePositionMax - Constants.Shooter.shooterAnglePositionTower));
            speed =
                Constants.Shooter.shootSpeedTrench + 
                (percent * (Constants.Shooter.shootSpeedCorner - Constants.Shooter.shootSpeedTowerBack));
        }
        else
        {
            // bigger
            angle = Constants.Shooter.shooterAnglePositionMax;
            speed = Constants.Shooter.shootSpeedCorner;
        }
        System.out.println("autoShooterRPM: distance=" + distance + " percent=" + percent + " angle=" + angle + " speed=" + speed);
        setShooter( speed, angle );
    }

    private double deadBandLeftY() {
        double newValue = joystick.getLeftY();
        if ((newValue <= 0.08) && (newValue >= -0.08))
        {
            return 0.0;
        }
        else if( bumpSpeedPressedDriver || bumpSpeedPressedOperator )
        {
            if( newValue < 0 )
            {
                return( -1 * Constants.Drive.bumpSpeed );
            }
            else
            {
                return( Constants.Drive.bumpSpeed );
            }
        }
        else if (newValue < 0) 
        {
            return (-1*(newValue*newValue));
        }
        else
        {
            return newValue*newValue;
        }

    }
    
    public void limelightSlow( boolean disabled )
    {
        m_Vision.limelightSlow( disabled );
    }

    private void BumpSpeedSet( boolean newval, boolean byOperator )
    {
        if( byOperator )
        {
            bumpSpeedPressedOperator = newval;
        }
        else
        {
            bumpSpeedPressedDriver = newval;
        }
    }

    private void PickupSpeedSet( boolean newval, boolean byOperator )
    {
        if( byOperator )
            pickupSpeedPressedOperator = newval;
        else
            pickupSpeedPressedDriver = newval;
        
        if( pickupSpeedPressedDriver || pickupSpeedPressedOperator )
            MaxSpeed = Constants.Drive.pickupSpeed * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        else
            MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    }

    private void SlowModeSet( boolean newval )
    {
        if( newval ){
            MaxSpeed = 1.0;
            MaxAngularRate = RotationsPerSecond.of(0.15).in(RadiansPerSecond);
        }
        else{
            MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
            MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
        }
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically when no other Drivetrain command is scheduled            
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-deadBandLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-deadBandLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-deadBandRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )                    
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // SHOOT
        joystick.a()
            .whileTrue( 
                Shoot()
            )
            .onFalse(
                ShooterStop()
            );

        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // UNJAM
        joystick.y()
            .whileTrue( 
                Commands.runOnce( ()->m_Intake.setConveyorPercentOutput(-0.50))
                .andThen(Commands.runOnce( ()->m_Shooter.setKickerRPM(-2000)) )
                .andThen(Commands.runOnce( ()->m_Shooter.setShooterRPM(-2000) ) ) //Constants.Shooter.shootSpeed
            )
            .onFalse(
                Commands.runOnce( ()->m_Intake.setConveyorPercentOutput(0))
                .andThen(Commands.runOnce( ()->m_Shooter.setKickerRPM(0)) )
                .andThen(Commands.runOnce( ()->m_Shooter.setShooterRPM(0) )
            )
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        /*
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        */

        // Reset the field-centric heading on BACK button (below/left of controller power)
        joystick.back().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // Make an X out of the swerve wheels
        joystick.x()
            .whileTrue(drivetrain.applyRequest(() -> brake));

        joystick.leftBumper()
            .onTrue( Commands.runOnce( ()->BumpSpeedSet( true, false ) ) )
            .onFalse( Commands.runOnce( ()->BumpSpeedSet( false, false )) );
        joystick.rightBumper()
            .onTrue( Commands.runOnce( ()->PickupSpeedSet( true, false ) ) )
            .onFalse( Commands.runOnce( ()->PickupSpeedSet( false, false )) );
        joystick.axisGreaterThan(2, 0.5)
            .onTrue( Commands.runOnce( ()->SlowModeSet(true) ) )
            .onFalse( Commands.runOnce( ()->SlowModeSet(false)) ) ;
        joystick.axisGreaterThan(3, 0.5)
            .whileTrue( Commands.repeatingSequence( AutoAim() ) );





        // =========== OPERATOR JOYSTICK =============
        // =========== OPERATOR JOYSTICK =============
        // =========== OPERATOR JOYSTICK =============

        operatorJoystick.povUp()
            .onTrue( Commands.runOnce( ()->m_Intake.setUpDownPercentOutput(0.2) ) )
            .onFalse( Commands.runOnce( ()->m_Intake.setUpDownPercentOutput(0) ) );
        
        operatorJoystick.povDown()
            .onTrue( Commands.runOnce( ()->m_Intake.setUpDownPercentOutput(-0.2) ) )
            .onFalse( Commands.runOnce( ()->m_Intake.setUpDownPercentOutput(0) ) );

        operatorJoystick.a()
            .onTrue(Commands.runOnce( ()->autoShooterRPM() ));
        
        operatorJoystick.x()
            .onTrue(Commands.runOnce( ()->this.setShooter( Constants.Shooter.shootSpeedPointBlank, Constants.Shooter.shooterAnglePositionMin )));

        operatorJoystick.y()
            .onTrue(Commands.runOnce( ()->this.setShooter( Constants.Shooter.shootSpeedTowerFront, Constants.Shooter.shooterAnglePositionTower )));

        operatorJoystick.b()
            .onTrue(Commands.runOnce( ()->this.setShooter( Constants.Shooter.shootSpeedCorner, Constants.Shooter.shooterAnglePositionMax )));

        operatorJoystick.axisGreaterThan(1, 0.2)
            .onTrue( Commands.runOnce( ()->m_Shooter.setAnglePercentOutput(0.5) ) )
            .onFalse( Commands.runOnce( ()->m_Shooter.setAnglePercentOutput(0) ) );
        operatorJoystick.axisLessThan(1, -0.2)
            .onTrue( Commands.runOnce( ()->m_Shooter.setAnglePercentOutput(-0.5) ) )
            .onFalse( Commands.runOnce( ()->m_Shooter.setAnglePercentOutput(0) ) );

        operatorJoystick.axisLessThan(5,-0.2)
        .and(operatorJoystick.axisGreaterThan(5,-0.9))
            .whileTrue( Commands.runOnce( ()->m_Intake.setRollerPercentOutput(-Constants.Intake.rollersPercentMax * 0.5)) )
            .onFalse( Commands.runOnce( ()->m_Intake.setRollerPercentOutput(0) ) );

        operatorJoystick.axisLessThan(5,-0.9)
            .whileTrue( Commands.runOnce( ()->m_Intake.setRollerPercentOutput(-Constants.Intake.rollersPercentMax)) )
            .onFalse( Commands.runOnce( ()->m_Intake.setRollerPercentOutput(0) ) );

        operatorJoystick.axisGreaterThan(5,0.2)
        .and(operatorJoystick.axisLessThan(5,0.9))
            .whileTrue( Commands.runOnce( ()->m_Intake.setRollerPercentOutput(Constants.Intake.rollersPercentMax * 0.5)) )
            .onFalse( Commands.runOnce( ()->m_Intake.setRollerPercentOutput(0) ) );

        operatorJoystick.axisGreaterThan(5,0.9)
            .whileTrue( Commands.runOnce( ()->m_Intake.setRollerPercentOutput(Constants.Intake.rollersPercentMax)) )
            .onFalse( Commands.runOnce( ()->m_Intake.setRollerPercentOutput(0) ) );

        operatorJoystick.leftBumper()
            .onTrue( Commands.runOnce( ()->BumpSpeedSet( true, true ) ) )
            .onFalse( Commands.runOnce( ()->BumpSpeedSet( false, true )) );
        operatorJoystick.rightBumper()
            .onTrue( Commands.runOnce( ()->PickupSpeedSet( true, true ) ) )
            .onFalse( Commands.runOnce( ()->PickupSpeedSet( false, true )) );
    }

    public void setShooter( double speed, double angle )
    {
        shootingSpeed = speed;
        m_Shooter.setAnglePosition( angle );
        m_Shooter.setShooterRPM(shootingSpeed);
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        Command command;
        command = autoChooser.getSelected();
        String name = command.getName();
        if( name.startsWith("right ") )
        {
            // if the path starts with "right ", mirror it from a left path
            // i.e., name the left path "2 coral us", and make a dummy right path "right 2 coral us"
            // - this will skip the dummy path and mirror the left path instead
            System.out.println("Flipping auton path " + name.substring(6));
            command = new PathPlannerAuto( name.substring(6), true );
        }
        else
        {
            System.out.println("Using auton path " + name);
        }
        return command;
    }

    public void setHubPosition()
    {
        if( drivetrain.getAlliance() == Alliance.Red )
        {
            hubPosition = new Translation2d(11.90, 4.02);
        }
        else
        {
            hubPosition = new Translation2d(4.61, 4.02);
        }

    }

    public void stopControls()
    {
        m_Intake.setRollerPercentOutput(0);
        m_Intake.setUpDownPosition(Constants.Intake.upDownPositionDown);
        CommandScheduler.getInstance().schedule( ShooterStop() );
    }

    private Rotation2d autoAimAngle;
    double autoAimDegrees = 999.0;

    private double AutoAimCalculate()
    {
        double newValue;
        Pose2d robotPose = drivetrain.getState().Pose;
        Translation2d toTarget = hubPosition.minus(robotPose.getTranslation());
        autoAimAngle = new Rotation2d(Math.atan2(
            toTarget.getY(),
            toTarget.getX()
        ));
        Rotation2d currentAngle = robotPose.getRotation().plus(Rotation2d.k180deg);
        Rotation2d neededTurn = autoAimAngle.minus(currentAngle);
        autoAimDegrees = -neededTurn.getDegrees();

        // ignore anything crazy
        if( (autoAimDegrees < -90) || (autoAimDegrees > 90))
            newValue = 0;
        else
        {
            newValue = autoAimDegrees/20.0;
            if( newValue > 0.2 ) newValue = 0.2;
            else if( newValue < -0.2 ) newValue = -0.2;
        }
        newValue = newValue * MaxAngularRate;

        if( autoAimDegrees < 3.0 )
        {
            bAutoAimDone = true;
        }
        else
        {
            bAutoAimDone = false;
        }
        Logger.recordOutput( "Shooter/AutoAimDone", false );

        return newValue;
    }

    private boolean AutoAimDone()
    {
        return bAutoAimDone;
    }

    private void AutoAimClear()
    {
        bAutoAimDone = false;
    }

    private Rotation2d getPossumAngle()
    {
        return Rotation2d.k180deg;
    }
}

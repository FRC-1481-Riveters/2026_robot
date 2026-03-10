// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.subsystems.VisionSubsystem;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;






public class RobotContainer {
    private final SendableChooser<Command> autoChooser;

    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private double shootingSpeed = 1800;

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

    private boolean autoAimPressed = false;
    private boolean bumpSpeedPressedOperator = false;
    private boolean bumpSpeedPressedDriver = false;
    private boolean pickupSpeedPressedOperator = false;
    private boolean pickupSpeedPressedDriver = false;

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
    }

    private Command IntakeLower()
    {
        return 
            Commands.runOnce( ()->m_Intake.setUpDownPosition(Constants.Intake.upDownPositionDown) )
                .andThen( Commands.runOnce( ()->m_Intake.setRollerCommandPercent(-Constants.Intake.rollersPercentMax * 0.5) ) )
                .andThen( Commands.waitSeconds(1.5))
                .andThen( Commands.runOnce( ()->m_Intake.setRollerCommandPercent(-Constants.Intake.rollersPercentMax) ) );
    }

    private Command RollerStop()
    {
        return 
            Commands.runOnce( ()->m_Intake.setRollerCommandPercent(0) );
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
        .andThen(Commands.runOnce( ()->m_Intake.setRollerCommandPercent(-Constants.Intake.rollersPercentMax)))
        .andThen(Commands.waitSeconds(3.5))
        .andThen(Commands.runOnce( ()->m_Intake.setRollerCommandPercent(-0.3)))
        .andThen(Commands.runOnce( ()->m_Intake.setUpDownPosition( Constants.Intake.upDownPosition30Degrees ) ))
        .andThen(Commands.waitSeconds(10.0));
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
        if(autoAimPressed)
        {
            Pose2d robotPose = drivetrain.getState().Pose;
            Translation2d target = new Translation2d(11.90, 4.02);
            Translation2d toTarget = target.minus(robotPose.getTranslation());
            Rotation2d aimAngle = new Rotation2d(Math.atan2(
                toTarget.getY(),
                toTarget.getX()
            ));
            Rotation2d currentAngle = robotPose.getRotation().plus(Rotation2d.k180deg);
            Rotation2d neededTurn = aimAngle.minus(currentAngle);
            double degrees = -neededTurn.getDegrees();

            // ignore anything crazy
            if( (degrees < -90) || (degrees > 90))
                newValue = 0;
            else
            {
                newValue = degrees/20.0;
                if( newValue > 0.2 ) newValue = 0.2;
                else if( newValue < -0.2 ) newValue = -0.2;
            }

            return newValue;
        }
        else if ((newValue <= 0.08) && (newValue >= -0.08))
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

    private void AutoAimSet( boolean newval )
    {
        autoAimPressed = newval;
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
        if( newval )
            MaxSpeed = 1.0;
        else
            MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
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

        // Intake will execute this command periodically when no other Intake command is scheduled
        m_Intake.setDefaultCommand( m_Intake.rollerRequest( ()->clipRollers() ) );

        // Shooter will execute this command periodically when no other Shooter command is scheduled
        //!*!*!* TODO: m_Shooter.setDefaultCommand( m_Shooter.angleRequest( ()->clipRollers() ) );

        joystick.leftBumper()
            .onTrue( Commands.runOnce( ()->AutoAimSet(true) ) )
            .onFalse( Commands.runOnce( ()->AutoAimSet(false) ) ) ;
            
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
                Commands.runOnce( ()->m_Intake.setConveyorPercentOutput(0))
                .andThen(Commands.runOnce( ()->m_Intake.setRollerCommandPercent(0)))
                .andThen(Commands.waitSeconds(0.25))
                .andThen(Commands.runOnce( ()->m_Shooter.setKickerRPM(0)) )
                .andThen(Commands.waitSeconds(0.25))
                .andThen(Commands.runOnce( ()->m_Shooter.setShooterRPM(0) )
            )
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
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

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
            .onTrue( Commands.runOnce( ()->AutoAimSet(true) ) )
            .onFalse( Commands.runOnce( ()->AutoAimSet(false)) ) ;


        // =========== OPERATOR JOYSTICK =============
        // =========== OPERATOR JOYSTICK =============
        // =========== OPERATOR JOYSTICK =============

        operatorJoystick.povUp()
            .onTrue( Commands.runOnce( ()->m_Intake.setUpDownPercentOutput(0.2) ) )
            .onFalse( Commands.runOnce( ()->m_Intake.setUpDownPercentOutput(0) ) );
        
        operatorJoystick.povDown()
            .onTrue( Commands.runOnce( ()->m_Intake.setUpDownPercentOutput(-0.2) ) )
            .onFalse( Commands.runOnce( ()->m_Intake.setUpDownPercentOutput(0) ) );
       
        operatorJoystick.povLeft()
            .onTrue( Commands.runOnce( ()->m_Shooter.setAnglePosition( -16.5 )))
            .onFalse( Commands.runOnce( ()->m_Shooter.setAnglePercentOutput(0))) ;

        operatorJoystick.povRight()
            .onTrue( Commands.runOnce( ()->m_Shooter.setAnglePosition( -0.6 )))
            .onFalse( Commands.runOnce( ()->m_Shooter.setAnglePercentOutput(0))) ;

        operatorJoystick.a()
            .onTrue(Commands.runOnce( ()->m_Shooter.setShooterRPM(Constants.Shooter.shootSpeed) ))
            .onFalse(Commands.runOnce( ()->m_Shooter.setShooterRPM(0) ));
        
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

    public double clipRollers()
    {
        double temp;

        temp = m_Intake.getRollerCommandPercent();
        if( temp == 0.0 )
        {
            temp = operatorJoystick.getRightY();
            if (temp > Constants.Intake.rollersPercentMax) {
                temp = Constants.Intake.rollersPercentMax;
            }
            else if (temp < -Constants.Intake.rollersPercentMax) {
                temp = -Constants.Intake.rollersPercentMax;
            }
        }
        return temp;
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
}

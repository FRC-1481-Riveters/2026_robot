// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Rotation2d;
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

    private final Telemetry logger = new Telemetry(MaxSpeed);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final VisionSubsystem m_Vision = new VisionSubsystem(drivetrain);
    private final Intake m_Intake = new Intake();
    private final Shooter m_Shooter = new Shooter();
    private LoggedNetworkNumber shooterSpeed = new LoggedNetworkNumber("/Tuning/ShooterSpeed", Constants.Shooter.shootSpeed);
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

    public RobotContainer() {
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

    private double deadBandLeftX() {
        double newValue = joystick.getLeftX();
        if ((newValue <= 0.05) && (newValue >= -0.05))
        {
            if( autoAimPressed )
            {
                return 0.0; // TODO: turn based on the Limelight angle!
            }
            else
            {
                return 0.0;
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

    private double deadBandRightX() {
        double newValue = joystick.getRightX();
        if ((newValue <= 0.05) && (newValue >= -0.05))
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
        if ((newValue <= 0.05) && (newValue >= -0.05))
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
    

    private void AutoAimSet( boolean newval )
    {
        autoAimPressed = newval;
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-deadBandLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-deadBandLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-deadBandRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )                    
        );

        m_Intake.setDefaultCommand( m_Intake.rollerRequest( ()->operatorJoystick.getRightY() ) );

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
                Commands.runOnce( ()->m_Shooter.setShooterRPM(shooterSpeed.get()) ) 
                .andThen(Commands.waitUntil( m_Shooter::isVelocityWithinTolerance) )
                .andThen(Commands.waitSeconds(0.5))
                .andThen(Commands.runOnce( ()->m_Shooter.setKickerRPM(shooterSpeed.get())) )
                .andThen(Commands.runOnce( ()->m_Intake.setConveyorPercentOutput(conveyorSpeed.get())) )
            )
            .onFalse(
                Commands.runOnce( ()->m_Intake.setConveyorPercentOutput(0))
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
                Commands.runOnce( ()->m_Intake.setConveyorPercentOutput(-0.20))
                .andThen(Commands.runOnce( ()->m_Shooter.setKickerRPM(-500)) )
                .andThen(Commands.runOnce( ()->m_Shooter.setShooterRPM(-500) ) ) //Constants.Shooter.shootSpeed
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

        // Reset the field-centric heading on START button (below/left of controller power)
        //joystick.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        joystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // Make an X out of the swerve wheels
        joystick.x()
            .whileTrue(drivetrain.applyRequest(() -> brake));

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
            .onTrue(Commands.runOnce( ()->m_Shooter.setShooterRPM(Constants.Shooter.shootSpeed) ))
            .onFalse(Commands.runOnce( ()->m_Shooter.setShooterRPM(0) ));
        
        operatorJoystick.axisGreaterThan(2, 0.1)
            .onTrue( Commands.runOnce( ()->m_Shooter.setAnglePercentOutput(0.2) ) )
            .onFalse( Commands.runOnce( ()->m_Shooter.setAnglePercentOutput(0) ) );
        operatorJoystick.axisLessThan(2, -0.1)
            .onTrue( Commands.runOnce( ()->m_Shooter.setAnglePercentOutput(-0.2) ) )
            .onFalse( Commands.runOnce( ()->m_Shooter.setAnglePercentOutput(0) ) );
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

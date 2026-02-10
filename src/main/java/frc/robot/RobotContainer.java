// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

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

public class RobotContainer {
    private final SendableChooser<Command> autoChooser;

    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private final Telemetry logger = new Telemetry(MaxSpeed);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final VisionSubsystem m_Vision = new VisionSubsystem(drivetrain);

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final CommandXboxController joystick = new CommandXboxController(0);


    public RobotContainer() {
        configureBindings();
        for (int port = 5801; port <= 5809; port++) {
//            PortForwarder.add(port, "limelight-back.local", port);
            PortForwarder.add(port, "10.14.81.11", port);
        }

        // add limelight 3a LEFT
        PortForwarder.add(5811, "10.14.81.12", 5801);
        PortForwarder.add(5812, "10.14.81.12", 5802);
        PortForwarder.add(5813, "10.14.81.12", 5803);
        PortForwarder.add(5814, "10.14.81.12", 5804);
        PortForwarder.add(5815, "10.14.81.12", 5805);
        PortForwarder.add(5816, "10.14.81.12", 5806);
        PortForwarder.add(5817, "10.14.81.12", 5807);
        PortForwarder.add(5818, "10.14.81.12", 5808);
        PortForwarder.add(5819, "10.14.81.12", 5809);
        // RIGHT
        PortForwarder.add(5811, "10.14.81.13", 5801);
        PortForwarder.add(5812, "10.14.81.13", 5802);
        PortForwarder.add(5813, "10.14.81.13", 5803);
        PortForwarder.add(5814, "10.14.81.13", 5804);
        PortForwarder.add(5815, "10.14.81.13", 5805);
        PortForwarder.add(5816, "10.14.81.13", 5806);
        PortForwarder.add(5817, "10.14.81.13", 5807);
        PortForwarder.add(5818, "10.14.81.13", 5808);
        PortForwarder.add(5819, "10.14.81.13", 5809);

        autoChooser = AutoBuilder.buildAutoChooser("Nothing");
        SmartDashboard.putData("Auto Mode", autoChooser);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
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

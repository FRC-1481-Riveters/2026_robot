package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.List;

import com.ctre.phoenix6.configs.CommutationConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.RobotContainer;


public class Intake extends SubsystemBase {
    private static final AngularVelocity kVelocityTolerance = RPM.of(100);

    private final TalonFXS upDownMotor, rollerMotor;
    private TalonFXS upDownPWM, rollerPWM;
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private RobotContainer rc;

    private double dashboardTargetRPM = 500.0;

    public Intake( RobotContainer robotContainer ) {
        rc = robotContainer;
        upDownMotor = new TalonFXS(Constants.CAN_motor_intake_updown);
        rollerMotor = new TalonFXS(Constants.CAN_motor_intake_roller);

        configureMotor(upDownMotor, InvertedValue.CounterClockwise_Positive, 50, 40);
        configureMotor(rollerMotor, InvertedValue.Clockwise_Positive, 60, 40);

        SmartDashboard.putData(this);
    }

    private void configureMotor(TalonFXS motor, InvertedValue invertDirection, double statorCurrentLimit, double supplyCurrentLimit ) {
        final TalonFXSConfiguration config = new TalonFXSConfiguration()
            .withCommutation(
                new CommutationConfigs()
                    .withMotorArrangement(MotorArrangementValue.VORTEX_JST)
            )   
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(invertDirection)
                    .withNeutralMode(NeutralModeValue.Brake)
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(statorCurrentLimit))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(supplyCurrentLimit))
                    .withSupplyCurrentLimitEnable(true)
            )
            .withSlot0(
                new Slot0Configs()
                    .withKP(0.5)
                    .withKI(2)
                    .withKD(0)
                    .withKV(12.0 / RPM.of(6000).in(RotationsPerSecond)) // 12 volts when requesting max RPS
            );
        
        motor.getConfigurator().apply(config);
    }

    /* public void setRPM(double rpm) {
        for (final TalonFX motor : motors) {
            motor.setControl(
                velocityRequest
                    .withVelocity(RPM.of(rpm))
            );
        }
    } */

    public void setUpDownPercentOutput(double percentOutput) {
        upDownMotor.setControl(
            voltageRequest
                .withOutput(Volts.of(percentOutput * 12.0))
        );
        System.out.println("setUpDownPercentOutput: " + percentOutput);
    }

    
    public void setRollerPercentOutput(double percentOutput) {
        rollerMotor.setControl(
            voltageRequest
                .withOutput(Volts.of(percentOutput * 12.0))
        );
    }

    @Override
    public void periodic() {
        double percentOutput;
        percentOutput = rc.getOperatorRoller();
        setRollerPercentOutput( percentOutput );
        // TODO Auto-generated method stub
        super.periodic();
    }

    
    private void initSendable(SendableBuilder builder, TalonFXS motor, String name) {
        builder.addDoubleProperty(name + " RPM", () -> motor.getVelocity().getValue().in(RPM), null);
        builder.addDoubleProperty(name + " Stator Current", () -> motor.getStatorCurrent().getValue().in(Amps), null);
        builder.addDoubleProperty(name + " Supply Current", () -> motor.getSupplyCurrent().getValue().in(Amps), null);
        builder.addDoubleProperty(name + " Torque Current", () -> motor.getTorqueCurrent().getValue().in(Amps), null);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        // initSendable(builder, leftShooterMotor, "Left");
        // initSendable(builder, rightShooterMotor, "Right");
        builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);
        builder.addDoubleProperty("Dashboard RPM", () -> dashboardTargetRPM, value -> dashboardTargetRPM = value);
        builder.addDoubleProperty("Target RPM", () -> velocityRequest.getVelocityMeasure().in(RPM), null);
    }
}
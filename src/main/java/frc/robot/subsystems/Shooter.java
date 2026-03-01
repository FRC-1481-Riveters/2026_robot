package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.List;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private static final AngularVelocity kVelocityTolerance = RPM.of(100);

    private final TalonFX leftShooterMotor, rightShooterMotor;
    private final TalonFX kickerMotor;
    private final TalonFX angleMotor;
    private final List<TalonFX> motors;
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);
    
    public Shooter() {
        leftShooterMotor = new TalonFX(Constants.CAN_motor_shooter_left);
        rightShooterMotor = new TalonFX(Constants.CAN_motor_shooter_right);
        motors = List.of(leftShooterMotor, rightShooterMotor);

        kickerMotor = new TalonFX(Constants.CAN_motor_kicker);
        angleMotor = new TalonFX(Constants.CAN_motor_angle);

        configureMotor(leftShooterMotor, InvertedValue.Clockwise_Positive, false, 50, 60);
        configureMotor(rightShooterMotor, InvertedValue.CounterClockwise_Positive, false, 50, 60);
        configureMotor(kickerMotor, InvertedValue.Clockwise_Positive, false, 120, 100);
        configureMotor(angleMotor, InvertedValue.CounterClockwise_Positive, true, 15, 20);

        Logger.recordOutput("PdhTotalCurrent", 0.0 );
        Logger.recordOutput("PdhTotalEnergy", 0.0 );
        Logger.recordOutput("Shooter/ShooterLeftSpeed", 0.0 );
        Logger.recordOutput("Shooter/ShooterRightSpeed", 0.0);
        Logger.recordOutput("Shooter/ShooterSetPoint", 0.0 );
        Logger.recordOutput("Shooter/ShooterRightCurrent", 0.0 );
        Logger.recordOutput("Shooter/ShooterLeftCurrent", 0.0 );
        Logger.recordOutput("Shooter/KickerSpeed", 0.0 );
        Logger.recordOutput("Shooter/KickerSetPoint", 0.0 );
        Logger.recordOutput("Shooter/KickerCurrent", 0.0 );
        Logger.recordOutput("Shooter/AnglePosition", 0.0 );
        Logger.recordOutput("Shooter/AngleSetPoint", 0.0 );
        Logger.recordOutput("Shooter/AngleOutput", 0.0 );
        Logger.recordOutput("Shooter/AngleCurrent", 0.0 );

        SmartDashboard.putData(this);
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Shooter/ShooterLeftSpeed", leftShooterMotor.getVelocity().getValue() );
        Logger.recordOutput("Shooter/ShooterRightSpeed", rightShooterMotor.getVelocity().getValue() );
        Logger.recordOutput("Shooter/ShooterRightCurrent", rightShooterMotor.getTorqueCurrent().getValueAsDouble() );
        Logger.recordOutput("Shooter/ShooterLeftCurrent", leftShooterMotor.getTorqueCurrent().getValueAsDouble() );
        Logger.recordOutput("Shooter/KickerSpeed", kickerMotor.getVelocity().getValue() );
        Logger.recordOutput("Shooter/KickerCurrent", kickerMotor.getTorqueCurrent().getValueAsDouble() );

        super.periodic();
    }


    private void configureMotor(TalonFX motor, InvertedValue invertDirection, boolean brakeMode, double statorLimit, double supplyLimit ) {
        NeutralModeValue mode;

        if( brakeMode == true )
            mode = NeutralModeValue.Brake;
        else   
            mode = NeutralModeValue.Coast;

        final TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(invertDirection)
                    .withNeutralMode(mode)
            )
            .withVoltage(
                new VoltageConfigs()
                    .withPeakReverseVoltage(Volts.of(0))
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(statorLimit))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(supplyLimit))
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

    public Command angleRequest(Supplier<Double> joystick) 
    {
        return run( () -> this.setAnglePercentOutput( joystick.get() ) );
    }

    public void setShooterRPM(double rpm) {
        for (final TalonFX motor : motors) {
            motor.setControl(
                velocityRequest
                    .withVelocity(RPM.of(rpm))
            );
        }
        Logger.recordOutput("Shooter/ShooterSetPoint", rpm);
    }

    public void setPercentOutput(double percentOutput) {
        for (final TalonFX motor : motors) {
            motor.setControl(
                voltageRequest
                    .withOutput(Volts.of(percentOutput * 12.0))
            );
        }
    }

    public void setKickerRPM(double rpm) {       
        kickerMotor.setControl(
            velocityRequest
                .withVelocity(RPM.of(rpm)));
        Logger.recordOutput("Shooter/KickerSetPoint", rpm );        
    }

    public void setAnglePercentOutput(double percentOutput) {
        double volts;
        if( percentOutput < 0.1 )
            percentOutput = 0;
        else
        {
            if( percentOutput > 0.4 ) percentOutput = 0.4;
            if( percentOutput < -0.4 ) percentOutput = -0.4;
        }

        volts = percentOutput * 12.0;
        angleMotor.setControl(
            voltageRequest
                .withOutput(Volts.of(volts))
        );
        Logger.recordOutput("Shooter/AngleOutput", volts );
    }

    public void stop() {
        setPercentOutput(0.0);
    }


    public boolean isVelocityWithinTolerance() {
        return motors.stream().allMatch(motor -> {
            final boolean isInVelocityMode = motor.getAppliedControl().equals(velocityRequest);
            final AngularVelocity currentVelocity = motor.getVelocity().getValue();
            final AngularVelocity targetVelocity = velocityRequest.getVelocityMeasure();
            return isInVelocityMode && currentVelocity.isNear(targetVelocity, kVelocityTolerance);
        });
    }
}
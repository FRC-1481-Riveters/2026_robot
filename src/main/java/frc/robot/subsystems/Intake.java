package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CommutationConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;


public class Intake extends SubsystemBase {

    private final TalonFXS upDownMotor, rollerInnerMotor, rollerOuterMotor, conveyorMotor;
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);

    public Intake() {
        upDownMotor = new TalonFXS(Constants.CAN_motor_intake_updown);
        rollerInnerMotor = new TalonFXS(Constants.CAN_motor_intake_roller_inner);
        rollerOuterMotor = new TalonFXS(Constants.CAN_motor_intake_roller_outer);
        conveyorMotor = new TalonFXS(Constants.CAN_motor_intake_conveyor);

        configureMotor(upDownMotor, InvertedValue.CounterClockwise_Positive, 50, 40);
        configureMotor(rollerInnerMotor, InvertedValue.Clockwise_Positive, 50, 60);
        configureMotor(rollerOuterMotor, InvertedValue.Clockwise_Positive, 50, 60);
        rollerOuterMotor.setControl(new Follower( rollerInnerMotor.getDeviceID(), MotorAlignmentValue.Opposed ) );

        configureMotor(conveyorMotor, InvertedValue.Clockwise_Positive, 50, 40);
        
        Logger.recordOutput("Intake/upDownPosition", 0 );
        Logger.recordOutput("Intake/upDownCurrent", 0 );
        Logger.recordOutput("Intake/upDownSetPoint", 0 );
        Logger.recordOutput("Intake/RollerInnerSpeed", 0 );
        Logger.recordOutput("Intake/RollerInnerCurrent", 0 );
        Logger.recordOutput("Intake/RollerOuterSpeed", 0 );
        Logger.recordOutput("Intake/RollerOuterCurrent", 0 );
        Logger.recordOutput("Intake/RollerSetPoint", 0 );
        Logger.recordOutput("Intake/ConveyorSpeed", 0 );
        Logger.recordOutput("Intake/ConveyorCurrent", 0 );
        Logger.recordOutput("Intake/ConveyorSetPoint", 0 );

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
                    .withKP(2)
                    .withKI(0)
                    .withKD(0)
                    .withKV(12.0 / RPM.of(3600).in(RotationsPerSecond)) // 12 volts when requesting max RPS
            );
        
        motor.getConfigurator().apply(config);
    }


    public void setUpDownPercentOutput(double percentOutput) 
    {
        double volts;
        volts = 12 * percentOutput;
        upDownMotor.setControl(
            voltageRequest
                .withOutput(Volts.of(volts))
        );
        Logger.recordOutput("Intake/UpDownSetPoint", volts );
    }

    
    public void setRollerPercentOutput(double percentOutput) {
        double volts;
        if( Math.abs(percentOutput) < 0.1 )
        {
            volts = 0;
        }
        else
        {
            volts = 12 * percentOutput;
        }

        rollerInnerMotor.setControl(
            voltageRequest
                .withOutput(Volts.of(volts))
        );

        Logger.recordOutput("Intake/RollerSetPoint", volts );

            /*
        else
            conveyorMotor.setControl(
                velocityRequest
                    .withVelocity(RPM.of(60 * percentOutput))
            );
            */

            /*
        else
            rollerMotor.setControl(
                velocityRequest
                    .withVelocity(RPM.of(60 * percentOutput))
            );
            */
    }

    public void setConveyorPercentOutput(double percentOutput) {
        double volts;
        if( Math.abs(percentOutput) < 0.1 )
        {
            volts = 0;
        }
        else
        {
            volts = 12 * percentOutput;
        }

        conveyorMotor.setControl(
            voltageRequest
                .withOutput(Volts.of(volts))
        );

        Logger.recordOutput("Intake/ConveyorSetPoint", volts );

            /*
        else
            conveyorMotor.setControl(
                velocityRequest
                    .withVelocity(RPM.of(60 * percentOutput))
            );
            */
    }

    public Command rollerRequest(Supplier<Double> joystick) 
    {
        return run(() -> this.setRollerPercentOutput(joystick.get()) );
    }

    @Override
    public void periodic() {

        Logger.recordOutput("Intake/UpDownPosition", upDownMotor.getPosition().getValueAsDouble() );
        Logger.recordOutput("Intake/UpDownCurrent", upDownMotor.getTorqueCurrent().getValueAsDouble() );
        Logger.recordOutput("Intake/RollerInnerSpeed", rollerInnerMotor.getVelocity().getValueAsDouble() );
        Logger.recordOutput("Intake/RollerInnerCurrent", rollerInnerMotor.getTorqueCurrent().getValueAsDouble() );
        Logger.recordOutput("Intake/RollerOuterSpeed", rollerOuterMotor.getVelocity().getValueAsDouble() );
        Logger.recordOutput("Intake/RollerOuterCurrent", rollerOuterMotor.getTorqueCurrent().getValueAsDouble() );
        Logger.recordOutput("Intake/ConveyorSpeed", conveyorMotor.getVelocity().getValueAsDouble() );
        Logger.recordOutput("Intake/ConveyorCurrent", conveyorMotor.getTorqueCurrent().getValueAsDouble() );

        super.periodic();
    }

    
}
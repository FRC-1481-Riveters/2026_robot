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
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;


public class Intake extends SubsystemBase {

    private final TalonFXS upDownMotor, rollerInnerMotor, rollerOuterMotor, conveyorMotor;
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final DigitalInput upDownLimitSwitch = new DigitalInput(0);

    public Intake() {
        upDownMotor = new TalonFXS(Constants.CAN_motor_intake_updown);
        rollerInnerMotor = new TalonFXS(Constants.CAN_motor_intake_roller_inner);
        rollerOuterMotor = new TalonFXS(Constants.CAN_motor_intake_roller_outer);
        conveyorMotor = new TalonFXS(Constants.CAN_motor_intake_conveyor);

        configureMotor(upDownMotor, InvertedValue.CounterClockwise_Positive, 50,60);
        configureMotor(rollerInnerMotor, InvertedValue.Clockwise_Positive, 70, 80);
        configureMotor(rollerOuterMotor, InvertedValue.Clockwise_Positive, 70, 80);
        rollerOuterMotor.setControl(new Follower( rollerInnerMotor.getDeviceID(), MotorAlignmentValue.Aligned ) );

        upDownMotor.setPosition(0);
        configureMotor(conveyorMotor, InvertedValue.Clockwise_Positive, 50, 40);
 
        // AdvantageKit inputs
        Logger.recordOutput("Intake/upDownPosition", 0.0 );
//        Logger.recordOutput("Intake/upDownCurrent", 0.0 );
        Logger.recordOutput("Intake/RollerInnerSpeed", 0.0 );
//        Logger.recordOutput("Intake/RollerInnerCurrent", 0.0 );
        Logger.recordOutput("Intake/RollerOuterSpeed", 0.0 );
//        Logger.recordOutput("Intake/RollerOuterCurrent", 0.0 );
//        Logger.recordOutput("Intake/ConveyorCurrent", 0.0 );
        Logger.recordOutput("Intake/ConveyorSetPoint", 0.0 );
        // AdvantageKit outputs
        Logger.recordOutput("Intake/upDownSetPoint", 0.0 );
        Logger.recordOutput("Intake/upDownLimitSwitch", false );
        Logger.recordOutput("Intake/RollerSetPoint", 0.0 );
        Logger.recordOutput("Intake/ConveyorSpeed", 0.0);

        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = 0.67; // An error of 1 rotation results in 2.4 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.0; // A velocity of 1 rps results in 0.1 V output
        slot0Configs.kV = 2.0;
        upDownMotor.getConfigurator().apply(slot0Configs);

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

    public void setUpDownPosition (double position) 
    {
        final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
        upDownMotor.setControl(m_request.withPosition(position));
        Logger.recordOutput("Intake/UpDownSetPoint", position );
    }

    
    public void setRollerPercentOutput(double percentOutput) {
        double volts;
        System.out.println("setRollerPercentOutput " + percentOutput);
        if( Math.abs(percentOutput) < 0.1 )
        {
            volts = 0;
        }
        else
        {
//            percentOutput /= 2.0;   // 50% is our ideal running speed (maximum torque)
            volts = 12 * percentOutput;
        }

        rollerInnerMotor.setControl(
            voltageRequest
                .withOutput(Volts.of(volts))
        );

        Logger.recordOutput("Intake/RollerSetPoint", volts );
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

    boolean upDownLimitPrevious = true;

    @Override
    public void periodic() {

        boolean upDownLimit = upDownLimitSwitch.get();
        if( upDownLimit == false && (upDownLimit != upDownLimitPrevious) )
        {
            upDownMotor.setPosition(Constants.Intake.upDownPositionDown);
        }
        upDownLimitPrevious = upDownLimit;
        Logger.recordOutput("Intake/UpDownPosition", upDownMotor.getPosition().getValueAsDouble() );
//        Logger.recordOutput("Intake/UpDownCurrent", upDownMotor.getTorqueCurrent().getValueAsDouble() );
        Logger.recordOutput("Intake/upDownLimitSwitch", upDownLimit );
        Logger.recordOutput("Intake/RollerInnerSpeed", rollerInnerMotor.getVelocity().getValueAsDouble() );
//        Logger.recordOutput("Intake/RollerInnerCurrent", rollerInnerMotor.getTorqueCurrent().getValueAsDouble() );
        Logger.recordOutput("Intake/RollerOuterSpeed", rollerOuterMotor.getVelocity().getValueAsDouble() );
//        Logger.recordOutput("Intake/RollerOuterCurrent", rollerOuterMotor.getTorqueCurrent().getValueAsDouble() );
        Logger.recordOutput("Intake/ConveyorSpeed", conveyorMotor.getVelocity().getValueAsDouble() );
//        Logger.recordOutput("Intake/ConveyorCurrent", conveyorMotor.getTorqueCurrent().getValueAsDouble() );

        super.periodic();
    }

    
}
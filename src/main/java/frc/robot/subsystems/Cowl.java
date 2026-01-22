package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Cowl extends SubsystemBase {
    private TalonFX cowl_motor;

    DutyCycleOut dc;

    private final DoubleSubscriber outputSubscriber = DogLog.tunable("Cowl/TunableCowlOutput", 0.1);

    double storedOutput = outputSubscriber.get();

    
    public Cowl() {
        cowl_motor = new TalonFX(13);

        dc = new DutyCycleOut(0);

        TalonFXConfiguration cowlConfiguration = new TalonFXConfiguration();
        cowlConfiguration.Feedback.SensorToMechanismRatio = 1;
       //  cowlConfiguration.MotorOutput.Inverted = false; change as needed
        cowl_motor.getConfigurator().apply(cowlConfiguration);
        cowl_motor.setNeutralMode(NeutralModeValue.Brake);

    }

    public Command outputForwardTunable() {
        return runEnd(() -> {
            cowl_motor.setControl(dc.withOutput(storedOutput));
        }, () -> {
            cowl_motor.set(0);
        });
    }

    public Command outputBackwardTunable() {
        return runEnd(() -> {
            cowl_motor.setControl(dc.withOutput(-storedOutput));
        }, () -> {
            cowl_motor.set(0);
        });
    }

    public Command output(double output) {
        return runEnd(() -> {
            cowl_motor.set(output);
        }, () -> {
            cowl_motor.set(0);
        });
    }

    public Command zeroPosition() {
        return runOnce(()-> {
            cowl_motor.setPosition(0);
        });
    }
 
    @Override
    public void periodic() {
        DogLog.log("Cowl/Position", cowl_motor.getPosition().getValueAsDouble());
        DogLog.log("Cowl/AppliedOutput", cowl_motor.getMotorVoltage().getValueAsDouble());
        DogLog.log("Cowl/Temperature", cowl_motor.getDeviceTemp().getValueAsDouble());

        storedOutput = outputSubscriber.get();  
    }
}

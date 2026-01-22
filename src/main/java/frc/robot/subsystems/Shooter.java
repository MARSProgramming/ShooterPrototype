package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private TalonFX master;
    private TalonFX follower;  
    private final IntegerSubscriber shooterVelocitySubscriber = DogLog.tunable("Shooter/TunableShooterVelocity", 2000);
    private final DoubleSubscriber percentOutSubscriber = DogLog.tunable("Shooter/TunableShooterOutput", 0.1);

    double storedNewVelocity = shooterVelocitySubscriber.get();
    double storedpercentOut = percentOutSubscriber.get(); 

    DutyCycleOut dc;
    VelocityVoltage vl;
    MotionMagicVelocityVoltage mmvv;
    
    public Shooter() {
        dc = new DutyCycleOut(0);
        vl = new VelocityVoltage(0);
        mmvv = new MotionMagicVelocityVoltage(0);
    
        master = new TalonFX(9);
        follower = new TalonFX(10); 

        TalonFXConfiguration shooterConfig = new TalonFXConfiguration();

        shooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        shooterConfig.Slot0.kP = 0.5;
        shooterConfig.Slot0.kI = 2;
        shooterConfig.Slot0.kD = 0;
        shooterConfig.Slot0.kV = 12.0 / RPM.of(6000).in(Units.RotationsPerSecond);

        master.getConfigurator().apply(shooterConfig);
        follower.getConfigurator().apply(shooterConfig);

        follower.setControl(new Follower(9, MotorAlignmentValue.Opposed));

    }

    public Command setOutput(double output) {
        return runEnd(() -> {
            master.setControl(dc.withOutput(output));
        }, () -> {
            master.set(0);
        });
    }

    
    public Command setOutputTunable() {
        return runEnd(() -> {
            master.setControl(dc.withOutput(storedpercentOut));
        }, () -> {
            master.set(0);
        });
    }

    
    public Command setVelocity(double velocity) {
        return runEnd(() -> {
            master.setControl(vl.withVelocity(RPM.of(velocity)));
        }, () -> {
            master.set(0);
        });
    }

    public Command setVelocityTunable() {
        return runEnd(() -> {
            master.setControl(vl.withVelocity(RPM.of(storedNewVelocity)));
        }, () -> {
            master.set(0);
        });
    }

    
    @Override
    public void periodic() {
        DogLog.log("Shooter/Master/VelocityRPM", RotationsPerSecond.of(master.getVelocity().getValueAsDouble()).in(Units.RPM));
        DogLog.log("Shooter/Follower/VelocityRPM", RotationsPerSecond.of(follower.getVelocity().getValueAsDouble()).in(Units.RPM));

        DogLog.log("Shooter/Master/AppliedVoltage", master.getMotorVoltage().getValueAsDouble());
        DogLog.log("Shooter/Follower/AppliedVoltage", master.getMotorVoltage().getValueAsDouble());

        DogLog.log("Shooter/Master/Temperature", master.getDeviceTemp().getValueAsDouble());
        DogLog.log("Shooter/Follower/Temperature", follower.getDeviceTemp().getValueAsDouble());

        storedNewVelocity = shooterVelocitySubscriber.get();
        storedpercentOut = percentOutSubscriber.get();
    }
}

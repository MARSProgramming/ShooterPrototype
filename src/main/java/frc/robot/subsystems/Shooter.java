package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
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
    
    public Shooter() {
        dc = new DutyCycleOut(0);
        vl = new VelocityVoltage(0);
    
        master = new TalonFX(9);
        follower = new TalonFX(10); 

        master.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
        follower.setControl(new Follower(9, MotorAlignmentValue.Opposed));

        // tune later
        Slot0Configs slot0 = new Slot0Configs()
                    .withKP(0.5)
                    .withKI(2)
                    .withKD(0)
                    .withKV(12.0 / RPM.of(6000).in(Units.RotationsPerSecond)); // 12 volts when requesting max RPS



        master.getConfigurator().apply(slot0);
        follower.getConfigurator().apply(slot0);

        master.setNeutralMode(NeutralModeValue.Coast);
        follower.setNeutralMode(NeutralModeValue.Coast);
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
        DogLog.log("Master/Velocity", master.getVelocity().getValueAsDouble());

        DogLog.log("Master/VelocityRecalculated", RotationsPerSecond.of(master.getVelocity().getValueAsDouble()).in(Units.RPM));

        DogLog.log("Master/AppliedOutput", master.getMotorVoltage().getValueAsDouble());
        DogLog.log("Master/Temperature", master.getDeviceTemp().getValueAsDouble());
        DogLog.log("Follower/Temperature", follower.getDeviceTemp().getValueAsDouble());

        storedNewVelocity = shooterVelocitySubscriber.get();
        storedpercentOut = percentOutSubscriber.get();
    }
}

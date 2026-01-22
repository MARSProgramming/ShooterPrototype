// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Cowl;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
  CommandXboxController pilot;
  Cowl cowl;
  Shooter shooter;

  public RobotContainer() {
    cowl = new Cowl();
    shooter = new Shooter();

    pilot = new CommandXboxController(0);
    configureBindings();
  }

  private void configureBindings() {
    pilot.rightBumper().whileTrue(cowl.outputForwardTunable());
    pilot.leftBumper().whileTrue(cowl.outputBackwardTunable());

    pilot.leftTrigger().whileTrue(shooter.setOutputTunable());
    pilot.rightTrigger().whileTrue(shooter.setVelocityTunable());

    pilot.a().onTrue(cowl.zeroPosition());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

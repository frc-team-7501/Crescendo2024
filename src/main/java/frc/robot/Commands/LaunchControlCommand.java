// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Launcher;

public class LaunchControlCommand extends Command {
  /** Creates a new LaunchCommand. */
  private final Launcher Launcher;
  private double launchSpeedDoubleT;
  private double launchSpeedDoubleB;

  public LaunchControlCommand(Launcher Launcher, Double launchSpeedDoubleT, Double launchSpeedDoubleB) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.Launcher = Launcher;
    this.launchSpeedDoubleT = launchSpeedDoubleT;
    this.launchSpeedDoubleB = launchSpeedDoubleB;
    addRequirements(Launcher);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (launchSpeedDoubleT == 0) {
      Launcher.stopLauncher(launchSpeedDoubleT, launchSpeedDoubleB);
    } else {
      Launcher.fireLauncher(launchSpeedDoubleT, launchSpeedDoubleB);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Launcher.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

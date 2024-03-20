// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Launcher;

public class AutonLauncherCommand extends Command {
  private Launcher launcher;
  private double velocity;
  /** Creates a new AutonLauncherCommand. */
  public AutonLauncherCommand(final Launcher launcher, final double velocity) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(launcher);
    this.launcher = launcher;
    this.velocity = velocity;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    launcher.fireLauncher(velocity, velocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}

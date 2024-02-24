// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Handoff;

public class AutonHandoffCommand extends Command {
  private Handoff handoff;
  private double handoffSpeed;
  /** Creates a new AutonHandoffCommand. */
  public AutonHandoffCommand(final Handoff handoff, final double handoffSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(handoff);
    this.handoff = handoff;
    this.handoffSpeed = handoffSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    handoff.fireHandoff(handoffSpeed, false, false);
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

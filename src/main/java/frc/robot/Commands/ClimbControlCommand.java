// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Climb;

public class ClimbControlCommand extends Command {
  /** Creates a new ClimbControlCommand. */
  private final Climb Climb;
  double ClimbPower;

  public ClimbControlCommand(Climb Climb, Double ClimbPower) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.Climb = Climb;
    addRequirements(Climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Climb.moveClimb(ClimbPower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

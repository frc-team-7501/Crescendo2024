// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.IntakeExtend;

public class IntakeExtendControlCommand extends Command {
    private final IntakeExtend intakeExtend;

  /** Creates a new IntakeExtendControlCommand. */
  public IntakeExtendControlCommand(final IntakeExtend intakeExtend) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeExtend);
    this.intakeExtend = intakeExtend;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeExtend.IntakeIn();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeExtend.IntakeOut();
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

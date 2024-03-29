// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Sensors;

public class IntakeControlCommand extends Command {
  /** Creates a new IntakeCommand. */
  private final Intake Intake;
  private double intakeSpeedDouble;
  private Sensors sensors;

  public IntakeControlCommand(Intake Intake, Double intakeSpeedDouble, Sensors sensors) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.Intake = Intake;
    this.intakeSpeedDouble = intakeSpeedDouble;
    this.sensors = sensors;
    addRequirements(Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (intakeSpeedDouble > 0) {
      Intake.runIntake(intakeSpeedDouble, sensors.getHandOffSensor());
    } else {
      Intake.runIntake(intakeSpeedDouble, false);
    } 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

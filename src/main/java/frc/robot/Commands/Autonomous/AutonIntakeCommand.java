// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Sensors;

public class AutonIntakeCommand extends Command {
  private Intake intake;
  private double intakeSpeedDouble;
  private Sensors sensors;
  /** Creates a new AutonIntakeCommand. */
  public AutonIntakeCommand(final Intake intake, final double intakeSpeedDouble, final Sensors sensors) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
    this.intake = intake;
    this.intakeSpeedDouble = intakeSpeedDouble;
    this.sensors = sensors;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.runIntake(intakeSpeedDouble, sensors.getHandOffSensor());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
     intake.runIntake(0, sensors.getHandOffSensor());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (sensors.getHandOffSensor()) {
      return true;
    } else {
      return false;
    }
  }
}

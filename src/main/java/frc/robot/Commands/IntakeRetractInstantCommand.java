// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.MiscMapping;
import frc.robot.Subsystems.IntakeExtend;
import frc.robot.Subsystems.Sensors;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeRetractInstantCommand extends InstantCommand {
  private final IntakeExtend intakeExtend;
  private final Sensors sensors;
  public IntakeRetractInstantCommand(final IntakeExtend intakeExtend, Sensors sensors) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeExtend);
    this.intakeExtend = intakeExtend;
    this.sensors = sensors;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeExtend.IntakeIn();
  }
}

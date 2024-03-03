// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.MiscMapping;
import frc.robot.Subsystems.ArmLift;
import frc.robot.Subsystems.Sensors;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmLiftPIDControlCommand extends PIDCommand {
  private final ArmLift armLift;
  private final boolean useTolerance;

  /** Creates a new ArmLiftPIDControlCommand. */
  public ArmLiftPIDControlCommand(final ArmLift armLift, final double position, final Sensors sensors, final boolean useTolerance) {
    super(
        // The controller that the command will use
        new PIDController(2.5, 0.5, 0),
        // This should return the measurement
        () -> sensors.getArmEncoderAbsolute(),
        // This should return the setpoint (can also be a constant)
        () -> position,
        // This uses the output
        output -> { 
          // Use the output here
          armLift.moveArm(-output);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    addRequirements(armLift);
    this.armLift = armLift;
    this.useTolerance = useTolerance;

    getController().setTolerance(MiscMapping.ARM_PID_TOLERANCE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (useTolerance) {
      return getController().atSetpoint();
    } else {
      return false;
    }
  }
}

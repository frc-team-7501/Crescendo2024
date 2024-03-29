// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Handoff;
import frc.robot.Subsystems.Sensors;

public class AutonHandoffCommand extends Command {
  private Handoff handoff;
  private double handoffSpeed;
  private Sensors sensors;
  private boolean override;
  private Timer currentTimer = new Timer();

  /** Creates a new AutonHandoffCommand. */
  public AutonHandoffCommand(final Handoff handoff, final double handoffSpeed, final Sensors sensors, boolean override) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(handoff);
    this.handoff = handoff;
    this.handoffSpeed = handoffSpeed;
    this.sensors = sensors;
    this.override = override;
    currentTimer.start();

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      currentTimer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (sensors.getHandOffSpeedSensor()) {
      handoffSpeed = handoffSpeed / 2;
      // handoffSpeed = 0;
    }
    handoff.fireHandoff(handoffSpeed, override, sensors.getHandOffSensor());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // handoff.fireHandoff(0.0, interrupted, true);
    handoff.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putNumber("Timer", currentTimer.get());
    if (sensors.getHandOffSensor() || override) {
      return true;
   } else {
      return false;
    }
  }
}

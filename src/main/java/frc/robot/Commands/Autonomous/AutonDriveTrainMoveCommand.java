// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Autonomous;

import frc.robot.Subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;

public class AutonDriveTrainMoveCommand extends Command {

    private final Drivetrain drivetrain;
    private double relativeSetpoint = 0;
    private double maxSpeed;

    public AutonDriveTrainMoveCommand(Drivetrain drivetrain, double setpoint, double maxSpeed) {
        this.drivetrain = drivetrain;
        this.relativeSetpoint = setpoint;
        this.maxSpeed = maxSpeed;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

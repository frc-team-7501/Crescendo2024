// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Autonomous;

import frc.robot.Subsystems.Drivetrain;
import frc.robot.utils.SimpleControllerSlow;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutonDriveTrainMoveCommand extends Command {

    private final SimpleControllerSlow controller;
    private final DriveTrain driveTrain;
    private double relativeSetpoint = 0;
    private double maxSpeed;

    public AutonDriveTrainMoveCommand(DriveTrain driveTrain, double setpoint, double maxSpeed) {
        this.driveTrain = driveTrain;
        this.relativeSetpoint = setpoint;
        this.maxSpeed = maxSpeed;

        addRequirements(driveTrain);
        controller = new SimpleControllerSlow(0.01, 0.05, driveTrain::getLeftDistance,
                (output) -> driveTrain.drive(output, .056, true), maxSpeed);

        controller.setTolerance(5);
    }

    @Override
    public void initialize() {
        controller.setSetpoint(driveTrain.getLeftDistance() + relativeSetpoint);
        controller.enable();
    }

    @Override
    public void execute() {
        controller.execute();
    }

    @Override
    public void end(boolean interrupted) {
        controller.disable();
    }

    @Override
    public boolean isFinished() {
        return controller.atSetpoint();
    }
}

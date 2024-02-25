package frc.robot.Commands.Autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Drivetrain;

public class AutonDriveCommand extends Command {
    private final Drivetrain drivetrain;
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController angleController;
    private final Pose2d targetPose2d;

    public AutonDriveCommand(final Drivetrain drivetrain, final Pose2d targetPose2d) {
        super();

        this.drivetrain = drivetrain;
        this.targetPose2d = targetPose2d;

        xController = Constants.DriveTrain.PID_X.toPidController();
        yController = Constants.DriveTrain.PID_Y.toPidController();
        angleController = Constants.DriveTrain.PID_T.toPidController();

        xController.setTolerance(2);
        yController.setTolerance(2);
        angleController.setTolerance(Math.toRadians(4));
    }

    @Override
    public void initialize() {
        xController.setSetpoint(targetPose2d.getX());
        yController.setSetpoint(targetPose2d.getY());
        angleController.setSetpoint(targetPose2d.getRotation().getRadians());
    }

    private double clamp(double val) {
        return Math.signum(val) * Math.max(Math.abs(val), 0.4);
    }

    @Override
    public void execute() {
        Pose2d currentPose = drivetrain.getPose();

        double outputX = xController.calculate(currentPose.getX());
        // double outputX = 0;
        double outputY = yController.calculate(currentPose.getY());
        // double outputY = 0;
        double outputT = angleController.calculate(currentPose.getRotation().getRadians());
        // double outputT = 0;

        SmartDashboard.putNumber("outputX", outputX);
        SmartDashboard.putNumber("outputY", outputY);
        SmartDashboard.putNumber("outputT", outputT);

        SmartDashboard.putBoolean("x atSetpoint", xController.atSetpoint());
        SmartDashboard.putBoolean("y atSetpoint", yController.atSetpoint());
        SmartDashboard.putBoolean("angle atSetpoint", angleController.atSetpoint());



        drivetrain.drive(-clamp(outputX), -clamp(outputY), -clamp(outputT), true);
    }

    @Override
    public boolean isFinished() {
        return xController.atSetpoint() && yController.atSetpoint() && angleController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("AutonDriveCommand.end() called!");
        // drivetrain.drive(0, 0, 0, true);
        drivetrain.stop();
    }
}

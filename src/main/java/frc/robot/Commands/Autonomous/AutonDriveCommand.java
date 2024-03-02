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

        xController = Constants.DriveTrain.PID_X.toPIDController();
        yController = Constants.DriveTrain.PID_Y.toPIDController();
        angleController = Constants.DriveTrain.PID_T.toPIDController();

        angleController.enableContinuousInput(-Math.PI, Math.PI); // TODO: what limits does pigeon give??
    }

    @Override
    public void initialize() {
        final double xConversionInches = 39.2/77; // 39.2 units / 77 inches
        final double yConversionInches = -39.47/78; // negate so left is negative, right is positive

        xController.setSetpoint(targetPose2d.getX() * xConversionInches);
        yController.setSetpoint(targetPose2d.getY() * yConversionInches);
        angleController.setSetpoint(targetPose2d.getRotation().getRadians());
    }

    private double clampOutput(double val, double limit) {
        return Math.signum(val) * Math.min(Math.abs(val), limit);
    }

    @Override
    public void execute() {
        Pose2d currentPose = drivetrain.getPose();

        double outputX = xController.calculate(currentPose.getX()); 
        // double outputX = 0;
        //double outputY = yController.calculate(currentPose.getY());
         double outputY = 0;
        // double outputT = angleController.calculate(currentPose.getRotation().getRadians());
        double outputT = 0;

        SmartDashboard.putNumber("outputX", outputX);
        SmartDashboard.putNumber("outputY", outputY);
        SmartDashboard.putNumber("outputT", outputT);

        drivetrain.driveRawFieldRelative(-clampOutput(outputX, 0.2), clampOutput(outputY, 0.2), clampOutput(outputT, 0.75));
    }

    @Override
    public boolean isFinished() {
        
        SmartDashboard.putBoolean("x atSetpoint", xController.atSetpoint());
        SmartDashboard.putBoolean("y atSetpoint", yController.atSetpoint());
        SmartDashboard.putBoolean("angle atSetpoint", angleController.atSetpoint());

        boolean condition = xController.atSetpoint() && yController.atSetpoint() && angleController.atSetpoint();

        SmartDashboard.putBoolean("condition", condition);

        return condition;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("AutonDriveCommand.end() called!");
        // drivetrain.drive(0, 0, 0, true);
        drivetrain.stop();
    }
}

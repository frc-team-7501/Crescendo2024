package frc.robot.Commands.Autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.MiscMapping;
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
        xController.setSetpoint(targetPose2d.getX() * MiscMapping.xConversionInches);
        yController.setSetpoint(targetPose2d.getY() * MiscMapping.yConversionInches);
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
        double outputY = yController.calculate(currentPose.getY());
        //  double outputY = 0;
        double outputT = angleController.calculate(currentPose.getRotation().getRadians());
        // double outputT = 0;

        //SmartDashboard.putNumber("outputX", outputX);
        //SmartDashboard.putNumber("outputY", outputY);
        //SmartDashboard.putNumber("outputT", outputT);

        // drivetrain.driveRawFieldRelative
        outputX = clampOutput(outputX, 0.3); 
        outputY = clampOutput(outputY, 0.3); 
        outputT = clampOutput(outputT, 0.75);

        //.putNumber("Clamped outputX", outputX);
        //SmartDashboard.putNumber("Clamped outputY", outputY);
        //SmartDashboard.putNumber("Clamped outputT", outputT);

        drivetrain.drive(-outputX, -outputY, -outputT, true, 1);
    }

    @Override
    public boolean isFinished() {
        
        SmartDashboard.putBoolean("x atSetpoint", xController.atSetpoint());
        SmartDashboard.putBoolean("y atSetpoint", yController.atSetpoint());
        SmartDashboard.putBoolean("angle atSetpoint", angleController.atSetpoint());

        boolean condition = xController.atSetpoint() && yController.atSetpoint() && angleController.atSetpoint();

        //SmartDashboard.putBoolean("condition", condition);

        return condition;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("AutonDriveCommand.end() called!");
        // drivetrain.drive(0, 0, 0, true);
        drivetrain.stop();
    }
}

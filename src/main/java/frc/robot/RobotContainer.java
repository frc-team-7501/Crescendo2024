// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ControllerMapping;
import frc.robot.Constants.MiscMapping;
import frc.robot.Commands.HandoffControlCommand;
import frc.robot.Commands.IntakeControlCommand;
import frc.robot.Commands.LaunchControlCommand;
import frc.robot.Commands.ResetGyroYawInstantCommand;
import frc.robot.Commands.SetIsFieldCentricInstantCommand;
import frc.robot.Commands.SetSpeedMultiplierInstantCommand;
import frc.robot.Commands.SwerveDriveManualCommand;
import frc.robot.Commands.Autonomous.AutonDriveCommand;
import frc.robot.Commands.Autonomous.AutonHandoffCommand;
import frc.robot.Commands.Autonomous.AutonIntakeCommand;
import frc.robot.Commands.Autonomous.AutonLauncherCommand;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Handoff;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Launcher;
import frc.robot.Subsystems.Sensors;
import frc.robot.utils.ExtendedXboxController;

public class RobotContainer {
    private final ExtendedXboxController m_Xbox = new ExtendedXboxController(ControllerMapping.XBOX);
    private final ExtendedXboxController m_Xbox2 = new ExtendedXboxController(ControllerMapping.XBOX2);

    // create subsystems
    private final Drivetrain driveTrain = Drivetrain.getInstance();
    private final Launcher launcher = Launcher.getInstance();
    private final Intake intake = Intake.getInstance();
    private final Handoff handoff = Handoff.getInstance();
    private final Sensors sensors = Sensors.getInstance();
    // private final Climb climb = Climb.getInstance();

    ////////////////////////////////
    // #region [ AUTON COMMANDS ]
    // #region Placeholder
    // Auton placeholder
    private final Command DefaultAuton = new SequentialCommandGroup(
            // Spin up launcher (1.5 seconds)
            new AutonLauncherCommand(launcher, MiscMapping.LAUNCH_VELOCITY),

            new WaitCommand(1.5),

            // Fire for 1.0 second
            new AutonHandoffCommand(handoff, MiscMapping.HANDOFF_SPEED, sensors, true),
            new WaitCommand(1.0),
            // Stop launcher/handoff after fire.
            new AutonHandoffCommand(handoff, 0.0, sensors, true),
            new AutonLauncherCommand(launcher, 0.0),

            // Spin intake and handoff until a Note is in the launcher and drive to pickup
            // point.
            new ParallelCommandGroup(
                    new AutonDriveCommand(driveTrain, new Pose2d(60, 0, new Rotation2d(0))),
                    new AutonIntakeCommand(intake, MiscMapping.INTAKE_SPEED, sensors),
                    new AutonHandoffCommand(handoff, MiscMapping.HANDOFF_SPEED, sensors, true)),

            // Stop intake and handoff once Note is in-place
            new AutonIntakeCommand(intake, 0.0, sensors),
            new AutonHandoffCommand(handoff, 0.0, sensors, false),

            // Spin up launcher (wait 1.5 seconds for ramp-up) and drive to shooting
            // position.
            new ParallelCommandGroup(
                    new SequentialCommandGroup(
                            new AutonLauncherCommand(launcher, MiscMapping.LAUNCH_VELOCITY),
                            new WaitCommand(1.5)),
                    new AutonDriveCommand(driveTrain, new Pose2d(0, 0, new Rotation2d(0)))),
            // Fire (1.0 second)
            new AutonHandoffCommand(handoff, MiscMapping.HANDOFF_SPEED, sensors, true),
            new WaitCommand(1.0),
            // Stop handoff and launcher after fire.
            new AutonHandoffCommand(handoff, 0.0, sensors, true),
            new AutonLauncherCommand(launcher, 0.0),

            // Spin intake and handoff until a Note is in the launcher and drive to pickup
            // point.
            new ParallelCommandGroup(
                    new AutonDriveCommand(driveTrain,
                            new Pose2d(60, 60, new Rotation2d((Math.PI / 180) * -50))),
                    new AutonIntakeCommand(intake, MiscMapping.INTAKE_SPEED, sensors),
                    new AutonHandoffCommand(handoff, MiscMapping.HANDOFF_SPEED, sensors, true)),

            // Stop intake and handoff once Note is in-place.
            new AutonIntakeCommand(intake, 0.0, sensors),
            new AutonHandoffCommand(handoff, 0.0, sensors, false),

            // Spin up launcher (wait 1.5 seconds for ramp-up) and drive to shooting
            // position.
            new ParallelCommandGroup(
                    new SequentialCommandGroup(
                            new AutonLauncherCommand(launcher, MiscMapping.LAUNCH_VELOCITY),
                            new WaitCommand(1.5)),
                    new AutonDriveCommand(driveTrain, new Pose2d(0, 0, new Rotation2d(0)))),
            // Fire (1.0 second)
            new AutonHandoffCommand(handoff, MiscMapping.HANDOFF_SPEED, sensors, true),
            new WaitCommand(1.0),
            // Stop handoff and launcher after fire
            new AutonHandoffCommand(handoff, 0.0, sensors, true),
            new AutonLauncherCommand(launcher, 0.0),

            new AutonDriveCommand(driveTrain, new Pose2d(60, 0, new Rotation2d(0))));

    // #endregion
    // #region
    private static class LaunchAuton extends SequentialCommandGroup {
        public LaunchAuton(Sensors sensors, Handoff handoff) {
            super(
                    new WaitCommand(0.25),
                    // Fire for 1.0 second
                    new AutonHandoffCommand(handoff, MiscMapping.HANDOFF_SPEED, sensors, true),
                    new WaitCommand(0.5),
                    // Stop launcher/handoff after fire
                    new AutonHandoffCommand(handoff, 0.0, sensors, true));
        }
    }
    // #endregion
    // #region Two Note Right
    private final Command TwoNoteRight = new SequentialCommandGroup(
            new InstantCommand(
                    () -> driveTrain.resetOdometry(new Pose2d(0, 0, new Rotation2d(0))),
                    driveTrain),
            new ParallelCommandGroup(
                new AutonLauncherCommand(launcher, MiscMapping.LAUNCH_VELOCITY),
                new LaunchAuton(sensors, handoff)),
                    new SequentialCommandGroup(
                    new ParallelCommandGroup(
                            new AutonDriveCommand(driveTrain,
                                    new Pose2d(62, -50, new Rotation2d((Math.PI / 180) * 40))),
                            new AutonIntakeCommand(intake, MiscMapping.INTAKE_SPEED, sensors),
                            new AutonHandoffCommand(handoff, MiscMapping.HANDOFF_SPEED, sensors, false)),
                    // Stop handoff once Note is in-place.
                    new AutonHandoffCommand(handoff, 0.0, sensors, false)),
                new AutonDriveCommand(driveTrain, new Pose2d(20, 0, new Rotation2d(0))),
            // Move back to the center position and fire the Launcher.
            new SequentialCommandGroup(
                    new AutonDriveCommand(driveTrain, new Pose2d(2, 0, new Rotation2d(0))),
                    new LaunchAuton(sensors, handoff)),
                new AutonDriveCommand(driveTrain,
                new Pose2d(60, -60, new Rotation2d((Math.PI / 180) * 50))));    
    // #endregion
    // #region Two Note Left
    private final Command TwoNoteLeft = new SequentialCommandGroup(
            new InstantCommand(
                    () -> driveTrain.resetOdometry(new Pose2d(0, 0, new Rotation2d(0))),
                    driveTrain),
            new ParallelCommandGroup(
                new AutonLauncherCommand(launcher, MiscMapping.LAUNCH_VELOCITY),
                new LaunchAuton(sensors, handoff)),
                    new SequentialCommandGroup(
                    new ParallelCommandGroup(
                            new AutonDriveCommand(driveTrain,
                                    new Pose2d(60, 60, new Rotation2d((Math.PI / 180) * -50))),
                            new AutonIntakeCommand(intake, MiscMapping.INTAKE_SPEED, sensors),
                            new AutonHandoffCommand(handoff, MiscMapping.HANDOFF_SPEED, sensors, false)),
                    // Stop handoff once Note is in-place.
                    new AutonHandoffCommand(handoff, 0.0, sensors, false)),
                new AutonDriveCommand(driveTrain, new Pose2d(20, 0, new Rotation2d(0))),
            // Move back to the center position and fire the Launcher.
            new SequentialCommandGroup(
                    new AutonDriveCommand(driveTrain, new Pose2d(2, 0, new Rotation2d(0))),
                    new LaunchAuton(sensors, handoff)),
                new AutonDriveCommand(driveTrain,
                new Pose2d(60, 60, new Rotation2d((Math.PI / 180) * -50))));    
    // #endregion
    // #region Four Note Auton
    private final Command FourNoteAuton = new SequentialCommandGroup(
            new InstantCommand(
                    () -> driveTrain.resetOdometry(new Pose2d(0, 0, new Rotation2d(0))),
                    driveTrain),
            new ParallelCommandGroup(
                    new AutonLauncherCommand(launcher, MiscMapping.LAUNCH_VELOCITY),
                    new LaunchAuton(sensors, handoff)),
            // Spin intake and handoff until a Note is in the launcher AND drive.
            new ParallelCommandGroup(
                    new AutonDriveCommand(driveTrain, new Pose2d(60, 5, new Rotation2d(0))),

                    new SequentialCommandGroup(
                            new ParallelCommandGroup(
                                    new AutonIntakeCommand(intake, MiscMapping.INTAKE_SPEED, sensors),
                                    new AutonHandoffCommand(handoff, MiscMapping.HANDOFF_SPEED / 2, sensors, true)),

                            // Stop handoff once Note is in-place.
                            new AutonHandoffCommand(handoff, 0.0, sensors, false))),
            // Move back to the center position and fire the Launcher.
            new SequentialCommandGroup(
                    new AutonDriveCommand(driveTrain, new Pose2d(2, 0, new Rotation2d(0))),
                    new LaunchAuton(sensors, handoff)),
            new SequentialCommandGroup(
                    new ParallelCommandGroup(
                            new AutonDriveCommand(driveTrain, new Pose2d(60, 55, new Rotation2d((Math.PI / 180) * -50))),
                            new AutonIntakeCommand(intake, MiscMapping.INTAKE_SPEED, sensors),
                            new AutonHandoffCommand(handoff, MiscMapping.HANDOFF_SPEED, sensors, false)),
                    // Stop handoff once Note is in-place.
                    new AutonHandoffCommand(handoff, 0.0, sensors, false)),
            // Move back to the center position and fire the Launcher.
            new SequentialCommandGroup(
                    new AutonDriveCommand(driveTrain, new Pose2d(2, 2, new Rotation2d(0))),
                    new LaunchAuton(sensors, handoff)),
            new SequentialCommandGroup(
                    new ParallelCommandGroup(
                            new AutonDriveCommand(driveTrain, new Pose2d(62, -53, new Rotation2d((Math.PI / 180) * 10))),
                            new AutonIntakeCommand(intake, MiscMapping.INTAKE_SPEED, sensors),
                            new AutonHandoffCommand(handoff, MiscMapping.HANDOFF_SPEED, sensors, false)),
             // Stop handoff once Note is in-place.
                    new AutonHandoffCommand(handoff, 0.0, sensors, false),
            // Move back to the center position and fire the Launcher.
            new SequentialCommandGroup(
                    new AutonDriveCommand(driveTrain, new Pose2d(2, -2, new Rotation2d(0))),
                    new LaunchAuton(sensors, handoff))));
    // #endregion
    // #region Three Note Base
    private final Command ThreeNoteAuton = new SequentialCommandGroup(
            new InstantCommand(
                    () -> driveTrain.resetOdometry(new Pose2d(0, 0, new Rotation2d(0))),
                    driveTrain),
            new ParallelCommandGroup(
                    new AutonLauncherCommand(launcher, MiscMapping.LAUNCH_VELOCITY),
                    new LaunchAuton(sensors, handoff)),

            new WaitCommand(1.0), // Wait for launcher to spin down.

            // Spin intake and handoff until a Note is in the launcher AND drive.
            new ParallelCommandGroup(
                    new AutonDriveCommand(driveTrain, new Pose2d(60, 0, new Rotation2d(0))),

                    new SequentialCommandGroup(
                            new ParallelCommandGroup(
                                    new AutonIntakeCommand(intake, MiscMapping.INTAKE_SPEED, sensors),
                                    new AutonHandoffCommand(handoff, MiscMapping.HANDOFF_SPEED / 2, sensors, true)),

                            // Stop handoff once Note is in-place.
                            new AutonHandoffCommand(handoff, 0.0, sensors, false))),
            // Move back to the center position and fire the Launcher.
            new SequentialCommandGroup(
                    new AutonDriveCommand(driveTrain, new Pose2d(0, 0, new Rotation2d(0))),
                    new LaunchAuton(sensors, handoff)),
            new SequentialCommandGroup(
                    new ParallelCommandGroup(
                            new AutonDriveCommand(driveTrain,
                                    new Pose2d(60, 55, new Rotation2d((Math.PI / 180) * -50))),
                            new AutonIntakeCommand(intake, MiscMapping.INTAKE_SPEED, sensors),
                            new AutonHandoffCommand(handoff, MiscMapping.HANDOFF_SPEED, sensors, false)),
                    // Stop handoff once Note is in-place.
                    new AutonHandoffCommand(handoff, 0.0, sensors, false)),
            // Move back to the center position and fire the Launcher.
            new SequentialCommandGroup(
                    new AutonDriveCommand(driveTrain, new Pose2d(0, 0, new Rotation2d(0))),
                    new LaunchAuton(sensors, handoff)),
            new AutonDriveCommand(driveTrain, new Pose2d(70, 0, new Rotation2d(0))));
    // #endregion
    // #endregion

    // Create commands
    // private final Command armLiftPIDControlCommand = new
    // ArmLiftPIDControlCommand(
    // armLift,
    // () -> MiscMapping.ARM_UP_POSITION);

    private final Command swerveDriveManualCommand = new SwerveDriveManualCommand(
            driveTrain,
            sensors,
            () -> m_Xbox.getLeftY(),
            () -> m_Xbox.getLeftX(),
            () -> m_Xbox.getRightX(),
            () -> sensors.getIsFieldCentric());

    // Joystick to control Climb.
    // private final Command climbControlCommand = new ClimbControlCommand(climb, ()
    // -> m_Xbox2.getLeftY());

    // private final InstantCommand ResetGyroYawInstantCommand = new
    // ResetGyroYawInstantCommand(
    // driveTrain);

    public RobotContainer() {
        configureButtonBindings();
        driveTrain.setDefaultCommand(swerveDriveManualCommand);
    }

    private void configureButtonBindings() {

        // Back button on the drive controller resets gyroscope.
        m_Xbox.b_Back().onTrue(new ResetGyroYawInstantCommand(driveTrain));

        // Button commands to intake the note.
        m_Xbox2.b_A()
                .onTrue(new ParallelCommandGroup(
                        new IntakeControlCommand(intake, MiscMapping.INTAKE_SPEED, sensors),
                        new HandoffControlCommand(handoff, sensors, MiscMapping.HANDOFF_SPEED, false)));

        m_Xbox2.b_A()
                .onFalse(new ParallelCommandGroup(
                        new IntakeControlCommand(intake, 0.0, sensors),
                        new HandoffControlCommand(handoff, sensors, 0.0, false)));

        // Button commands to launch the note.
        m_Xbox2.b_B()
                .onTrue(new LaunchControlCommand(launcher, MiscMapping.LAUNCH_VELOCITY));
        m_Xbox2.b_B()
                .onFalse(new LaunchControlCommand(launcher, 0.0));

        // Move Handoff as long as the button is pressed.
        m_Xbox2.b_RightBumper()
                .onTrue(new HandoffControlCommand(handoff, sensors, MiscMapping.HANDOFF_SPEED, true));
        m_Xbox2.b_RightBumper()
                .onFalse(new HandoffControlCommand(handoff, sensors, 0.0, false));

        // Reverse the velocity to reverse the Note.
        m_Xbox2.b_LeftBumper()
                .onTrue(new ParallelCommandGroup(
                        new IntakeControlCommand(intake, MiscMapping.REVERSE_INTAKE_SPEED, sensors),
                        new HandoffControlCommand(handoff, sensors, MiscMapping.REVERSE_HANDOFF_SPEED, false)));
        m_Xbox2.b_LeftBumper()
                .onFalse(new ParallelCommandGroup(
                        new IntakeControlCommand(intake, 0.0, sensors),
                        new HandoffControlCommand(handoff, sensors, 0.0, false)));

        // Turbo Button
        m_Xbox.b_RightBumper()
                .onTrue(new SetSpeedMultiplierInstantCommand(sensors, MiscMapping.TURBO_MULTIPLIER));
        m_Xbox.b_RightBumper()
                .onFalse(new SetSpeedMultiplierInstantCommand(sensors, MiscMapping.NORMAL_MULTIPLIER));

        // Field Centric Toggle
        m_Xbox.b_LeftBumper()
                .onTrue(new SetIsFieldCentricInstantCommand(sensors, false));
        m_Xbox.b_LeftBumper()
                .onFalse(new SetIsFieldCentricInstantCommand(sensors, true));
    }

    public void teleopInit() {
        driveTrain.setBrakeMode(MiscMapping.BRAKE_OFF);
    }

    public void autonomousInit() {
        driveTrain.setBrakeMode(MiscMapping.BRAKE_ON);
        driveTrain.resetYaw();
    }

    public Command getAutonomousCommand() {
        // [ MAIN AUTONS ]

        // var autonCommand = new SequentialCommandGroup(
        // new InstantCommand(
        // () -> driveTrain.resetOdometry(new Pose2d(0, 0, new Rotation2d(0))),
        // driveTrain
        // ),
        // new ParallelCommandGroup(
        // new ArmLiftPIDControlCommand(armLift, MiscMapping.ARM_UP_POSITION, sensors),
        // new SequentialCommandGroup(
        // new AutonDriveCommand(driveTrain, new Pose2d(60, -24, new
        // Rotation2d(Math.PI))),
        // new AutonDriveCommand(driveTrain, new Pose2d(0, 0, new Rotation2d(0)))
        // )
        // )
        // );

        // return autonCommand;
        // return DefaultAuton;

        // var autonCommand =
        // DefaultAuton

        // return DefaultAuton;
        // return ThreeNoteAuton;
        return FourNoteAuton;
        // return TwoNoteLeft;
        // return TwoNoteRight;
    }
}

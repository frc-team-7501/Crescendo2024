// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ControllerMapping;
import frc.robot.Constants.MiscMapping;
import frc.robot.Commands.ArmLiftPIDControlCommand;
//import frc.robot.Commands.ClimbControlCommand;
import frc.robot.Commands.HandoffControlCommand;
import frc.robot.Commands.IntakeControlCommand;
import frc.robot.Commands.IntakeExtendInstantCommand;
import frc.robot.Commands.IntakeRetractInstantCommand;
import frc.robot.Commands.LaunchControlCommand;
import frc.robot.Commands.ResetGyroYawInstantCommand;
import frc.robot.Commands.SetDeliverySelectorInstantCommand;
import frc.robot.Commands.SetSpeedMultiplierInstantCommand;
import frc.robot.Commands.SwerveDriveManualCommand;
import frc.robot.Commands.Autonomous.AutonDriveCommand;
import frc.robot.Commands.Autonomous.AutonHandoffCommand;
import frc.robot.Commands.Autonomous.AutonIntakeCommand;
import frc.robot.Commands.Autonomous.AutonLauncherCommand;
import frc.robot.Subsystems.ArmLift;
//import frc.robot.Subsystems.Climb;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Handoff;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.IntakeExtend;
import frc.robot.Subsystems.Launcher;
import frc.robot.Subsystems.Limelight;
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
        private final IntakeExtend intakeExtend = IntakeExtend.getInstance();
        private final ArmLift armLift = ArmLift.getInstance();
        private final Sensors sensors = Sensors.getInstance();
        private final Limelight limelight = Limelight.getInstance();
        // private final Climb climb = Climb.getInstance();

        // Variables to replace stick values on the Controller.
        // public double XPosition;
        // public double YPosition;
        // public double ZPosition;
        // public boolean ArmLiftRunnable;

        ////////////////////////////////
        // #region [ AUTON COMMANDS ]
        // #region Placeholder
        // Auton placeholder
        private final Command DefaultAuton = new SequentialCommandGroup(
                // Move arm to DOWN position
          
                // Spin up launcher (1.5 seconds)
                // new ParallelCommandGroup(
                // new ArmLiftPIDControlCommand(armLift, MiscMapping.ARM_UP_POSITION, sensors),
                new AutonLauncherCommand(launcher, MiscMapping.LAUNCH_VELOCITY),
                // ),
                new WaitCommand(1.5),
          
                // Fire for 1.0 second
                new AutonHandoffCommand(handoff, MiscMapping.HANDOFF_SPEED, sensors, true),
                new WaitCommand(1.0),
                // Stop launcher/handoff after fire
                new AutonHandoffCommand(handoff, 0.0, sensors, true),
                new AutonLauncherCommand(launcher, 0.0),
          
                // Move intake down to pick up
                // new ArmLiftPIDControlCommand(armLift, MiscMapping.ARM_DOWN_POSITION,
                // sensors),
          
                // new WaitCommand(2.0),
          
                // Spin intake and handoff until a Note is in the launcher and drive to pickup
                // point
                new ParallelCommandGroup(
                    new AutonDriveCommand(driveTrain, new Pose2d(60, 0, new Rotation2d(0))),
                    new AutonIntakeCommand(intake, MiscMapping.INTAKE_VELOCITY, sensors),
                    new AutonHandoffCommand(handoff, MiscMapping.HANDOFF_SPEED, sensors, true)),
                // Stop intake and handoff once Note is in-place
                new AutonIntakeCommand(intake, 0.0, sensors),
                new AutonHandoffCommand(handoff, 0.0, sensors, false),
          
                // Spin up launcher (wait 1.5 seconds for ramp-up) and drive to shooting
                // position
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
          
                // Spin intake and handoff until a Note is in the launcher and drive to pickup
                // point
                new ParallelCommandGroup(
                    new AutonDriveCommand(driveTrain, new Pose2d(60, 60, new Rotation2d((Math.PI/180) * -50))),
                    new AutonIntakeCommand(intake, MiscMapping.INTAKE_VELOCITY, sensors),
                    new AutonHandoffCommand(handoff, MiscMapping.HANDOFF_SPEED, sensors, true)),
                // Stop intake and handoff once Note is in-place
                new AutonIntakeCommand(intake, 0.0, sensors),
                new AutonHandoffCommand(handoff, 0.0, sensors, false),
          
                // Spin up launcher (wait 1.5 seconds for ramp-up) and drive to shooting
                // position
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
        //#region
        private final Command ForwardMovement = new ParallelCommandGroup(
                new AutonDriveCommand(driveTrain, new Pose2d(60, 0, new Rotation2d(0)))); 

        //#endregion

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
                        () -> MiscMapping.FIELD_RELATIVE);

        // Joystick to control Climb.
        // private final Command climbControlCommand = new ClimbControlCommand(climb, ()
        // -> m_Xbox2.getLeftY());

        // private final InstantCommand ResetGyroYawInstantCommand = new
        // ResetGyroYawInstantCommand(
        // driveTrain);

        public RobotContainer() {
                configureButtonBindings();
                driveTrain.setDefaultCommand(swerveDriveManualCommand);
                // climb.setDefaultCommand(climbControlCommand);
        }

        private void configureButtonBindings() {
                // Back button on the drive controller resets gyroscope.
                m_Xbox.b_Back().onTrue(new ResetGyroYawInstantCommand(driveTrain));

                // Button commands to intake the note.
                m_Xbox2.b_A().onTrue(new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                                new WaitCommand(0.7),
                                                new IntakeControlCommand(intake, MiscMapping.INTAKE_VELOCITY, sensors)),
                                new HandoffControlCommand(handoff, sensors, MiscMapping.HANDOFF_SPEED, false),
                                new ArmLiftPIDControlCommand(armLift, MiscMapping.ARM_DOWN_POSITION, sensors),
                                new IntakeRetractInstantCommand(intakeExtend, sensors)));

                m_Xbox2.b_A().onFalse(new ParallelCommandGroup(
                                new IntakeControlCommand(intake, 0.0, sensors),
                                new HandoffControlCommand(handoff, sensors, 0.0, false),
                                new ArmLiftPIDControlCommand(armLift, MiscMapping.ARM_UP_POSITION, sensors)));

                // Button commands to launch the note.
                m_Xbox2.b_B().onTrue(new LaunchControlCommand(launcher, MiscMapping.LAUNCH_VELOCITY));
                m_Xbox2.b_B().onFalse(new LaunchControlCommand(launcher, 0.0));

                m_Xbox2.b_RightBumper()
                                .onTrue(new ParallelCommandGroup(
                                                new HandoffControlCommand(handoff, sensors, MiscMapping.HANDOFF_SPEED,
                                                                true),
                                                new ArmLiftPIDControlCommand(armLift, MiscMapping.ARM_UP_POSITION,
                                                                sensors)));
                m_Xbox2.b_RightBumper()
                                .onFalse(new ParallelCommandGroup(
                                                new HandoffControlCommand(handoff, sensors, 0.0, false),
                                                new ArmLiftPIDControlCommand(armLift, MiscMapping.ARM_UP_POSITION,
                                                                sensors)));

                // Button commands to control for AMP.
                m_Xbox2.b_Y().onTrue(new IntakeExtendInstantCommand(intakeExtend, sensors));

                m_Xbox2.b_LeftBumper()
                                .onTrue(new ParallelCommandGroup(
                                                new IntakeControlCommand(intake, MiscMapping.AMP_VELOCITY, sensors),
                                                new ArmLiftPIDControlCommand(armLift, MiscMapping.ARM_UP_POSITION,
                                                                sensors)));
                m_Xbox2.b_LeftBumper().onFalse(new ParallelCommandGroup(new IntakeControlCommand(intake, 0.0, sensors),
                                new ArmLiftPIDControlCommand(armLift, MiscMapping.ARM_UP_POSITION, sensors)));

                // Choose either to deliver at the Amp or Speaker.
                m_Xbox.b_A().onTrue(new SetDeliverySelectorInstantCommand(sensors, false)); // Speaker
                m_Xbox.b_B().onTrue(new SetDeliverySelectorInstantCommand(sensors, true)); // Amp

                // "Phantom Button"
                m_Xbox2.b_X()
                                .onTrue(new ParallelCommandGroup(
                                                new ArmLiftPIDControlCommand(armLift, MiscMapping.ARM_PHANTOM_POSITION,
                                                                sensors),
                                                new IntakeRetractInstantCommand(intakeExtend, sensors)));
                m_Xbox2.b_X()
                                .onFalse(new ArmLiftPIDControlCommand(armLift, MiscMapping.ARM_UP_POSITION, sensors));

                // Turbo Button
                m_Xbox.b_RightBumper().onTrue(new SetSpeedMultiplierInstantCommand(sensors, MiscMapping.TURBO_MULTIPLIER));
                m_Xbox.b_RightBumper().onFalse(new SetSpeedMultiplierInstantCommand(sensors, MiscMapping.NORMAL_MULTIPLIER));
        }

        public void teleopInit() {
                driveTrain.setBrakeMode(MiscMapping.BRAKE_OFF);
                intakeExtend.IntakeIn();
                // CommandScheduler.getInstance().schedule(ArmLiftPIDControlCommand(armLift,
                // MiscMapping.ARM_UP_POSITION));
                var command = new ParallelCommandGroup(
                                new ArmLiftPIDControlCommand(armLift, MiscMapping.ARM_UP_POSITION, sensors));

                CommandScheduler.getInstance().schedule(command);
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
                return ForwardMovement;
        }
}

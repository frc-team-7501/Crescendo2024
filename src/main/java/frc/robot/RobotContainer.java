// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.*;
import frc.robot.utils.ExtendedXboxController;
import frc.robot.Commands.ArmLiftPIDControlCommand;
import frc.robot.Commands.HandoffControlCommand;
import frc.robot.Commands.IntakeControlCommand;
import frc.robot.Commands.IntakeExtendInstantCommand;
import frc.robot.Commands.IntakeRetractInstantCommand;
import frc.robot.Commands.LaunchControlCommand;
import frc.robot.Commands.ResetGyroYawInstantCommand;
import frc.robot.Commands.SetDeliverySelectorInstantCommand;
import frc.robot.Commands.SwerveDriveManualCommand;
import frc.robot.Commands.Autonomous.AutonHandoffCommand;
import frc.robot.Commands.Autonomous.AutonLauncherCommand;
import frc.robot.Constants.ControllerMapping;
import frc.robot.Constants.MiscMapping;

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

  public double XPosition;
  public double YPosition;
  public double ZPosition;
  public boolean ArmLiftRunnable;

  ////////////////////////////////
  // #region [ AUTON COMMANDS ]
  // #region Placeholder
  // Auton placeholder
  private final Command DefaultAuton = new SequentialCommandGroup(
      new AutonLauncherCommand(launcher, MiscMapping.LAUNCH_VELOCITY),
      new WaitCommand(1.5),
      new AutonHandoffCommand(handoff, MiscMapping.HANDOFF_SPEED),
      new WaitCommand(1.0),
      new AutonHandoffCommand(handoff, 0.0),
      new AutonLauncherCommand(launcher, 0.0));

  // #endregion
  // #endregion

  // Create commands
  // private final Command armLiftPIDControlCommand = new
  // ArmLiftPIDControlCommand(
  // armLift,
  // () -> MiscMapping.ARM_UP_POSITION);

  private final Command swerveDriveManualCommand = new SwerveDriveManualCommand(
      driveTrain,
      () -> m_Xbox.getLeftY(),
      () -> m_Xbox.getLeftX(),
      () -> m_Xbox.getRightX(),
      () -> MiscMapping.FIELD_RELATIVE);

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

    m_Xbox2.b_RightBumper().onTrue(new HandoffControlCommand(handoff, sensors, MiscMapping.HANDOFF_SPEED, true));
    m_Xbox2.b_RightBumper().onFalse(new HandoffControlCommand(handoff, sensors, 0.0, false));

    // Button commands to control for AMP.
    m_Xbox2.b_Y().onTrue(new IntakeExtendInstantCommand(intakeExtend, sensors));

    m_Xbox2.b_LeftBumper()
        .onTrue(new ParallelCommandGroup(new IntakeControlCommand(intake, MiscMapping.AMP_VELOCITY, sensors),
            new ArmLiftPIDControlCommand(armLift, MiscMapping.ARM_UP_POSITION, sensors)));
    m_Xbox2.b_LeftBumper().onFalse(new ParallelCommandGroup(new IntakeControlCommand(intake, 0.0, sensors),
        new ArmLiftPIDControlCommand(armLift, MiscMapping.ARM_UP_POSITION, sensors)));

    // Chose either to deliver at the Amp or Speaker.
    m_Xbox.b_A().onTrue(new SetDeliverySelectorInstantCommand(sensors, false)); // Speaker
    m_Xbox.b_B().onTrue(new SetDeliverySelectorInstantCommand(sensors, true)); // Amp
  }

  public void teleopInit() {
    driveTrain.setBrakeMode(MiscMapping.BRAKE_OFF);
    intakeExtend.IntakeIn();
    // CommandScheduler.getInstance().schedule(ArmLiftPIDControlCommand(armLift,
    // MiscMapping.ARM_UP_POSITION));
    new ParallelCommandGroup(new ArmLiftPIDControlCommand(armLift, MiscMapping.ARM_UP_POSITION, sensors));
  }

  public void autonomousInit() {
    driveTrain.setBrakeMode(MiscMapping.BRAKE_ON);
  }

  public Command getAutonomousCommand() {
    // [ MAIN AUTONS ]
    return DefaultAuton; // Just a wait command to satisfy WPILIB.
  }
}

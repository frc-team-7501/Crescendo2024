// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANMapping;

public class Launcher extends SubsystemBase {
  private final CANSparkMax m_LaunchMotorT = new CANSparkMax(CANMapping.LAUNCH_SPARKMAX_T, MotorType.kBrushless);
  private final CANSparkMax m_LaunchMotorB = new CANSparkMax(CANMapping.LAUNCH_SPARKMAX_B, MotorType.kBrushless);
  private final SparkPIDController m_LaunchPIDControllerT;
  private final SparkPIDController m_LaunchPIDControllerB;
  private final RelativeEncoder m_LaunchEncoderT;
  private final RelativeEncoder m_LaunchEncoderB;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
  private static Launcher instance;

  /** Creates a new Launcher. */
  public Launcher() {
    m_LaunchPIDControllerT = m_LaunchMotorT.getPIDController();
    m_LaunchPIDControllerB = m_LaunchMotorB.getPIDController();
    m_LaunchEncoderT = m_LaunchMotorT.getEncoder();
    m_LaunchEncoderB = m_LaunchMotorB.getEncoder();
    // TODO: MAKE THESE CONSTANTS
    kP = 1e-4;
    kI = 5e-7;
    kD = 1e-6;
    kIz = 0;
    kFF = 0.0000015;
    kMaxOutput = 0.8;
    kMinOutput = -0.8;
    maxRPM = 5700;

    m_LaunchPIDControllerT.setP(kP);
    m_LaunchPIDControllerT.setI(kI);
    m_LaunchPIDControllerT.setD(kD);
    m_LaunchPIDControllerT.setIZone(kIz);
    m_LaunchPIDControllerT.setFF(kFF);
    m_LaunchPIDControllerT.setOutputRange(kMinOutput, kMaxOutput);

    m_LaunchPIDControllerB.setP(kP);
    m_LaunchPIDControllerB.setI(kI);
    m_LaunchPIDControllerB.setD(kD);
    m_LaunchPIDControllerB.setIZone(kIz);
    m_LaunchPIDControllerB.setFF(kFF);
    m_LaunchPIDControllerB.setOutputRange(kMinOutput, kMaxOutput);
  }

  public static Launcher getInstance() {
    if (instance == null)
      instance = new Launcher();
    return instance;
  }

  public void fireLauncher(double velocityT, double velocityB) {
    m_LaunchPIDControllerT.setReference(velocityT, CANSparkMax.ControlType.kVelocity);
    m_LaunchPIDControllerB.setReference(velocityB, CANSparkMax.ControlType.kVelocity);
  }

  public void stopLauncher(double velocityT, double velocityB) {
    m_LaunchMotorT.set(velocityT);
    m_LaunchMotorB.set(velocityB);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("TopLV", m_LaunchEncoderT.getVelocity());
    SmartDashboard.putNumber("BotLV", m_LaunchEncoderB.getVelocity());
  }

  public void stop() {
    m_LaunchMotorT.stopMotor();
    m_LaunchMotorB.stopMotor();
  }
}

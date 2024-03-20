// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANMapping;

public class Launcher extends SubsystemBase {
  private final CANSparkMax m_LaunchMotorT = new CANSparkMax(CANMapping.LAUNCH_SPARKMAX_T, MotorType.kBrushless);
  private final CANSparkMax m_LaunchMotorB = new CANSparkMax (CANMapping.LAUNCH_SPARKMAX_B, MotorType.kBrushless);
  private static Launcher instance;

  /** Creates a new Launcher. */
  public Launcher() {}

  public static Launcher getInstance() {
    if (instance == null)
      instance = new Launcher();
    return instance;
  }

  public void fireLauncher(double velocityT, double velocityB) {
    m_LaunchMotorT.set(velocityT);
    m_LaunchMotorB.set(velocityB);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void stop() {
    m_LaunchMotorT.stopMotor();
    m_LaunchMotorB.stopMotor();
  }
}

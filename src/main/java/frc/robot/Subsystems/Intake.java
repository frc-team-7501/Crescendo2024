// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANMapping;

public class Intake extends SubsystemBase {
  private final CANSparkMax m_IntakeMotorT = new CANSparkMax(CANMapping.INTAKE_SPARKMAX_T, MotorType.kBrushless);
  private final CANSparkMax m_IntakeMotorB = new CANSparkMax(CANMapping.INTAKE_SPARKMAX_B, MotorType.kBrushless);
  private static Intake instance;

  /** Creates a new Intake. */
  public Intake() {}

  public static Intake getInstance() {
    if (instance == null)
      instance = new Intake();
    return instance;
  }

  public void runIntake(double velocity, boolean sensor) {
    if (sensor) {
      m_IntakeMotorT.set(0.0);
      m_IntakeMotorB.set(0.0);
    } else {
      m_IntakeMotorT.set(velocity);
      m_IntakeMotorB.set(velocity);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void stop() {
    m_IntakeMotorT.stopMotor();
    m_IntakeMotorB.stopMotor();
  }
}

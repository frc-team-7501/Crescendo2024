// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANMapping;
import frc.robot.Constants.DIOMapping;

public class ArmLift extends SubsystemBase {
  private final CANSparkMax m_ArmLiftMotor = new CANSparkMax(CANMapping.INTAKE_SPARKMAX_LIFT, MotorType.kBrushless);
  private static ArmLift instance;

  /** Creates a new ArmLift. */
  public ArmLift() {}

  public static ArmLift getInstance() {
    if (instance == null)
      instance = new ArmLift();
    return instance;
  }

  public void moveArm (double speed) {
    SmartDashboard.putNumber("Arm PID Speed", speed);
    m_ArmLiftMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void stop() {
    m_ArmLiftMotor.stopMotor();
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANMapping;
import frc.robot.Constants.DIOMapping;

public class Handoff extends SubsystemBase {
  private final TalonSRX m_HandoffMotor = new TalonSRX(CANMapping.HANDOFF_TALONSRX);
  private static Handoff instance;
  private DigitalInput handoffSensor = new DigitalInput(DIOMapping.HANDOFF_SENSOR);

  /** Creates a new Handoff. */
  public Handoff() {
  }

  public static Handoff getInstance() {
    if (instance == null)
      instance = new Handoff();
    return instance;
  }

  public void fireHandoff(double HandoffPower, boolean override) {
    if (override){
      m_HandoffMotor.set(TalonSRXControlMode.PercentOutput, HandoffPower);
    } else if (!handoffSensor.get()){
      m_HandoffMotor.set(TalonSRXControlMode.PercentOutput, 0.0);
    } else {
      m_HandoffMotor.set(TalonSRXControlMode.PercentOutput, HandoffPower);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void stop() {
    m_HandoffMotor.set(TalonSRXControlMode.PercentOutput, 0);
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DIOMapping;

public class Sensors extends SubsystemBase {
  /** Creates a new Sensors. */
  private DigitalInput handoffSpeedSensor = new DigitalInput(DIOMapping.HANDOFF_SPEED_SENSOR);
  private DigitalInput handoffSensor = new DigitalInput(DIOMapping.HANDOFF_SENSOR);
  private DigitalInput intakeSensor = new DigitalInput(DIOMapping.INTAKE_SENSOR);
  private DutyCycleEncoder armEncoder = new DutyCycleEncoder(DIOMapping.ARM_LIFT_ENCODER);
  private boolean deliverySelector;
  private static Sensors instance;


  public Sensors() {
  //Set the default delivery method to Launcher.
    deliverySelector = true;
  }

  public static Sensors getInstance() {
    if (instance == null)
      instance = new Sensors();
    return instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Handoff Sensor", getHandOffSensor());
    SmartDashboard.putBoolean("Intake Sensor", getIntakeSensor());
    SmartDashboard.putBoolean("Delivery Selector", getDeliverySelector());
    SmartDashboard.putNumber("Arm Position", getArmEncoderAbsolute());
    SmartDashboard.putBoolean("Handoff Speed Sensor", getHandOffSpeedSensor());
  }

  public boolean getHandOffSpeedSensor() {
    return !handoffSpeedSensor.get();
  }

  public boolean getHandOffSensor() {
    return !handoffSensor.get();
  }

  public boolean getIntakeSensor() {
    return !intakeSensor.get();
  }

  public boolean getDeliverySelector() {
    return deliverySelector;
  }

  public void setDeliverySelector(boolean selector) {
  deliverySelector = selector;
  }

  public double getArmEncoderAbsolute() {
    return armEncoder.getAbsolutePosition();
  }
}

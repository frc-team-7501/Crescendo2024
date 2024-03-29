// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DIOMapping;
import frc.robot.Constants.MiscMapping;

public class Sensors extends SubsystemBase {
  /** Creates a new Sensors. */
  private DigitalInput handoffSpeedSensor = new DigitalInput(DIOMapping.HANDOFF_SPEED_SENSOR);
  private DigitalInput handoffSensor = new DigitalInput(DIOMapping.HANDOFF_SENSOR);
  private DigitalInput intakeSensor = new DigitalInput(DIOMapping.INTAKE_SENSOR);
  // private boolean deliverySelector;
  private boolean isFieldCentric;
  private double speedMultiplier;
  private static Sensors instance;

  public Sensors() {
  //Set the default delivery method to Launcher.
    // deliverySelector = true;
    isFieldCentric = true;
    speedMultiplier = MiscMapping.NORMAL_MULTIPLIER;
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
    //SmartDashboard.putBoolean("Intake Sensor", getIntakeSensor());
    // SmartDashboard.putBoolean("Delivery Selector", getDeliverySelector());
    //SmartDashboard.putBoolean("Handoff Speed Sensor", getHandOffSpeedSensor());
    SmartDashboard.putBoolean("Field Centric", getIsFieldCentric());

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

  // public boolean getDeliverySelector() {
  //   return deliverySelector;
  // }

  // public void setDeliverySelector(boolean selector) {
  // deliverySelector = selector;
  // }

  public boolean getIsFieldCentric() {
    return isFieldCentric;
  }

  public void setIsFieldCentric(boolean isCentric) {
  isFieldCentric = isCentric;
  }
  
  public double getSpeedMultiplier() {
    return speedMultiplier;
  }

  public void setSpeedMultiplier(double multiplier) {
  speedMultiplier = multiplier;
  }
}

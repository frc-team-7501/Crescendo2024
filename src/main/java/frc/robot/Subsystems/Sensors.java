// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DIOMapping;
import frc.robot.Constants.MiscMapping;

public class Sensors extends SubsystemBase {
  /** Creates a new Sensors. */
  private DigitalInput handoffSpeedSensor = new DigitalInput(DIOMapping.HANDOFF_SPEED_SENSOR);
  private DigitalInput handoffSensor = new DigitalInput(DIOMapping.HANDOFF_SENSOR);
  // private boolean deliverySelector;
  private boolean isFieldCentric;
  private double speedMultiplier;
  private static Sensors instance;
  // Auton DIO Selector
  private DigitalInput auto1Input = new DigitalInput(DIOMapping.AUTON_1_INPUT);
  private DigitalInput auto2Input = new DigitalInput(DIOMapping.AUTON_2_INPUT);
  private DigitalInput auto3Input = new DigitalInput(DIOMapping.AUTON_3_INPUT);
  private DigitalInput auto4Input = new DigitalInput(DIOMapping.AUTON_4_INPUT);
  private DigitalInput auto5Input = new DigitalInput(DIOMapping.AUTON_5_INPUT);
  private DigitalInput autoAllianceInput = new DigitalInput(DIOMapping.AUTON_ALLIANCE_INPUT);
  // Pixie Sensor
  private DutyCycleEncoder pixySensorEncoer = new DutyCycleEncoder(DIOMapping.PIXY_SENSOR);

  public Sensors() {
    // Set the default delivery method to Launcher.
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
    // SmartDashboard.putBoolean("Delivery Selector", getDeliverySelector());
    // SmartDashboard.putBoolean("Handoff Speed Sensor", getHandOffSpeedSensor());
    SmartDashboard.putBoolean("Field Centric", getIsFieldCentric());
    SmartDashboard.putNumber("AutoSelector Value", getAutonSelected());
    SmartDashboard.putNumber("Pixy", getPixySensor());

  }

  public boolean getHandOffSpeedSensor() {
    return !handoffSpeedSensor.get();
  }

  public boolean getHandOffSensor() {
    return !handoffSensor.get();
  }

  // public boolean getDeliverySelector() {
  // return deliverySelector;
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

  public int getAutonSelected() {
      SmartDashboard.putBoolean("ASensor1", !auto1Input.get());
      SmartDashboard.putBoolean("ASensor2", !auto2Input.get());
      SmartDashboard.putBoolean("ASensor3", !auto3Input.get());
      SmartDashboard.putBoolean("ASensor4", !auto4Input.get());
      SmartDashboard.putBoolean("ASensor5", !auto5Input.get());

    if (!auto1Input.get()) {
      return 1;
    } else if (!auto2Input.get()) {
      return 2;
    } else if (!auto3Input.get()) {
      return 3;
    } else if (!auto4Input.get()) {
      return 4;
    } else if (!auto5Input.get()) {
      return 5;
    } else {
      return 0;
    }
  }

  public boolean getAutonAllianceSelected() {
    return !autoAllianceInput.get();
  }

public double getPixySensor() {
    return pixySensorEncoer.getAbsolutePosition();
  }
}

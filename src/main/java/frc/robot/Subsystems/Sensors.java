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
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Sensors extends SubsystemBase {
  /** Creates a new Sensors. */
  // Handoff Sensors
  private DigitalInput handoffSpeedSensor = new DigitalInput(DIOMapping.HANDOFF_SPEED_SENSOR);
  private DigitalInput handoffSensor = new DigitalInput(DIOMapping.HANDOFF_SENSOR);
  // Climb Limit Switches
  private DigitalInput climbLimitSwitch1 = new DigitalInput(DIOMapping.CLIMB_LIMIT_SWITCH_1);
  private DigitalInput climbLimitSwitch2 = new DigitalInput(DIOMapping.CLIMB_LIMIT_SWITCH_2);
  // Auton DIO Selector
  private DigitalInput auto1Input = new DigitalInput(DIOMapping.AUTON_1_INPUT);
  private DigitalInput auto2Input = new DigitalInput(DIOMapping.AUTON_2_INPUT);
  private DigitalInput auto3Input = new DigitalInput(DIOMapping.AUTON_3_INPUT);
  private DigitalInput auto4Input = new DigitalInput(DIOMapping.AUTON_4_INPUT);
  private DigitalInput auto5Input = new DigitalInput(DIOMapping.AUTON_5_INPUT);
  private DigitalInput autoAllianceInput = new DigitalInput(DIOMapping.AUTON_ALLIANCE_INPUT);
  // Pixie Sensor
  private DutyCycleEncoder pixySensorEncoder = new DutyCycleEncoder(DIOMapping.PIXY_SENSOR);
  // Photon Vision
  private PhotonCamera photonCamera = new PhotonCamera("Cam01"); 
  private double photonYaw;
  private double photonPitch;
  private PhotonTrackedTarget target;
  private int targetID;
  // Other "Fake" Sensors
  private boolean isFieldCentric;
  private double speedMultiplier;
  private static Sensors instance;

  public Sensors() {
    // Set the default delivery method to Launcher.
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
    //SmartDashboard.putNumber("AutoSelector Value", getAutonSelected());
    SmartDashboard.putNumber("Pixy", getPixySensor());
    SmartDashboard.putBoolean("Limit", getClimbLimitSwitch());

  }

  public boolean getHandOffSpeedSensor() {
    return !handoffSpeedSensor.get();
  }

  public boolean getHandOffSensor() {
    return !handoffSensor.get();
  }

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
    //SmartDashboard.putBoolean("ASensor1", !auto1Input.get());
    //SmartDashboard.putBoolean("ASensor2", !auto2Input.get());
    //SmartDashboard.putBoolean("ASensor3", !auto3Input.get());
    //SmartDashboard.putBoolean("ASensor4", !auto4Input.get());
    //SmartDashboard.putBoolean("ASensor5", !auto5Input.get());
    //SmartDashboard.putBoolean("Alliance", !autoAllianceInput.get());

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
    return pixySensorEncoder.getAbsolutePosition();
  }

  public double getPhotonVisionYaw() {
    var result = photonCamera.getLatestResult();
    SmartDashboard.putBoolean("hasTarget", result.hasTargets());
    if (result.hasTargets()) {
      // Sees a target
      target = result.getBestTarget();
      targetID = target.getFiducialId();
      SmartDashboard.putNumber("targetID", targetID);
      SmartDashboard.putNumber("photonYaw", target.getYaw());
      if (targetID == 5 || targetID == 6) {
        photonYaw = target.getYaw();
        return photonYaw;
      } else {
        return 0.0;
      }
    } else {
      // Doesn't see a target
      return 0.0;
    }
  }

  public double getPhotonVisionPitch() {
    var result = photonCamera.getLatestResult();
    SmartDashboard.putBoolean("hasTarget", result.hasTargets());
    if (result.hasTargets()) {
      // Sees a target
      target = result.getBestTarget();
      targetID = target.getFiducialId();
      SmartDashboard.putNumber("targetID", targetID);
      SmartDashboard.putNumber("photonPitch", target.getPitch());
      if (targetID == 5 || targetID == 6) {
        photonPitch = target.getPitch();
        return photonPitch - MiscMapping.PHOTON_PITCH_GOAL;
      } else {
        return 0.0;
      }
    } else {
      // Doesn't see a target
      return 0.0;
    }
  }

  public boolean getClimbLimitSwitch() {
    if (!climbLimitSwitch1.get() || !climbLimitSwitch2.get()) {
      return true;
    } else {
      return false;
    }
  }
}

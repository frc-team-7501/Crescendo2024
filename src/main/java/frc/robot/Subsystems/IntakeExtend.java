// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANMapping;
import frc.robot.Constants.PneumaticsMapping;

public class IntakeExtend extends SubsystemBase {
   private final Solenoid extendSolenoid = new Solenoid(CANMapping.PNEUMATIC_HUB, PneumaticsModuleType.REVPH,
      PneumaticsMapping.PNEUMATIC_SINGLE_SOLENOID_EXTEND);
  private final Solenoid retractSolenoid = new Solenoid(CANMapping.PNEUMATIC_HUB,PneumaticsModuleType.REVPH,
      PneumaticsMapping.PNEUMATIC_SINGLE_SOLENOID_RETRACT);
  
      private static IntakeExtend instance;

      public static IntakeExtend getInstance() {
        if (instance == null)
          instance = new IntakeExtend();
        return instance;
      }
    
      /** Creates a new IntakeExtend. */
  public IntakeExtend() {}

  public void IntakeIn() {
    // Schedules code to be delivered without disrupting other code
    //final ScheduledExecutorService executorService = Executors.newSingleThreadScheduledExecutor();

    extendSolenoid.set(false);
    retractSolenoid.set(true);

    // Waits a second without stopping any other code
    /*final Runnable retract = new Runnable(){
      public void run(){
        retractSolenoid.set(false);
      }
    };
    executorService.schedule(retract, 1000, TimeUnit.MILLISECONDS);*/
  }

  public void IntakeOut() {
    if (extendSolenoid.get()) {
      IntakeIn();
      //SmartDashboard.putBoolean("Call", true);
    } else {
      retractSolenoid.set(false);
      extendSolenoid.set(true);
      //SmartDashboard.putBoolean("Call", false);
    }
  }
  
  public void stop() {
    IntakeOut();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Periodic is called every 20ms, updating our SmartDashboard
    if(extendSolenoid.get()){
      SmartDashboard.putString("Extend/Retract", "Extend/Closed");
    } else{
      SmartDashboard.putString("Extend/Retract", "Retract/Open");
    }
    
  }
}
// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

 package frc.robot.Subsystems;

 import com.revrobotics.CANSparkMax;
 import com.revrobotics.CANSparkLowLevel.MotorType;
 import edu.wpi.first.wpilibj2.command.SubsystemBase;
 import frc.robot.Constants.CANMapping;

 public class Climb extends SubsystemBase {
   private final CANSparkMax m_ClimbMotor = new CANSparkMax(CANMapping.CLIMB_SPARKMAX, MotorType.kBrushless);
   private static Climb instance;
  
   // Creates a new Climb. 
   public Climb() {}

   public static Climb getInstance() {
     if (instance == null)
     instance = new Climb();
     return instance;
   }

   public void moveClimb(double ClimbPower) {
     m_ClimbMotor.set(ClimbPower);
   }

   @Override
   public void periodic() {
     // This method will be called once per scheduler run
   }

   public void stop() {
     m_ClimbMotor.set(0.0);
   }
}

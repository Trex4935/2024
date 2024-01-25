// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.extension.SparkMax;

public class Shooter extends SubsystemBase {

   //Declaring Motors
  CANSparkMax shootingmotor1;
  CANSparkMax shootingmotor2;
  CANSparkMax magazinemotor;
 
 
  // Makes a new state for the shooter


  /** Creates a new Shooter. */
  public Shooter() {

    //Creating Motor Objects
    shootingmotor1 = SparkMax.createDefaultCANSparkMax(16);
    shootingmotor2 = SparkMax.createDefaultCANSparkMax(26);
    magazinemotor = SparkMax.createDefaultCANSparkMax(36);

   

}
 //makes motors spin YIPPIE
  public void shooterMovement(){
    shootingmotor1.set(-0.8);
    shootingmotor2.set(0.8);
  }
//makes magazine motor spin
   public void magazineMotorMovement(){
    magazinemotor.set(0.5);
  }

//stops motors
  public void stopShootingMotor1(){
    shootingmotor1.stopMotor();
  }
  public void stopShootingMotor2(){
    shootingmotor2.stopMotor();
  }
  public void stopMagazineMotor(){
    magazinemotor.stopMotor();
  }
 
  public void stopAllMotors(){
   stopShootingMotor1();
   stopShootingMotor2();
   stopMagazineMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  
}

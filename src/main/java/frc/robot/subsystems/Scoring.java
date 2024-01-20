// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.extension.SparkMax;

public class Scoring extends SubsystemBase {

  //Declaring Motors
  CANSparkMax shootingmotor1;
  CANSparkMax shootingmotor2;
  CANSparkMax magazinemotor;
  CANSparkMax pivotMotor;

  /** Creates a new Scoring. */
  public Scoring() {

    //Creating Motor Objects
    shootingmotor1 = SparkMax.createDefaultCANSparkMax(1);
    shootingmotor2 = SparkMax.createDefaultCANSparkMax(2);
    magazinemotor = SparkMax.createDefaultCANSparkMax(3);
    pivotMotor = SparkMax.createDefaultCANSparkMax(6);
  
}
//makes motors spin YIPPIE
  public void scoringMovement(){
    shootingmotor1.set(-0.9);
    shootingmotor2.set(0.9);
  }
//makes magazine motor spin
   public void magazineMotorMovement(){
    magazinemotor.set(0.5);
  }
//makes pivot motor move
  public void pivotMotor(){
    pivotMotor.set(0);
    pivotMotor.getPIDController().setReference(180, CANSparkBase.ControlType.kPosition);
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
  public void stopPivotMotor(){
    pivotMotor.stopMotor();
  }
  public void stopAllMotors(){
   stopShootingMotor1();
   stopShootingMotor2();
   stopMagazineMotor();
   stopPivotMotor();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

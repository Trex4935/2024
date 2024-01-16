// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.extension.SparkMax;

public class Scoring extends SubsystemBase {

  //Declaring Motors
  CANSparkMax shootingmotor1;
  CANSparkMax shootingmotor2;
  CANSparkMax magazinemotor;

  /** Creates a new Scoring. */
  public Scoring() {

    //Creating Motor Objects
    shootingmotor1 = SparkMax.createDefaultCANSparkMax(1);
    shootingmotor2 = SparkMax.createDefaultCANSparkMax(2);
    magazinemotor = SparkMax.createDefaultCANSparkMax(3);
  
}
//makes motors spin YIPPIE
  public void scoringMovement(){
    shootingmotor1.set(0.5);
    shootingmotor2.set(0.5);
    magazinemotor.set(0.5);
  }
//stops motors
  public void stopScoringMotor(){
    shootingmotor1.stopMotor();
    shootingmotor2.stopMotor();
    magazinemotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

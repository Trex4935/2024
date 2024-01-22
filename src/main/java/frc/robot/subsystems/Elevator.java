// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.extension.SparkMax;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  CANSparkMax elevatorMotorOne;
  CANSparkMax elevatorMotorTwo;

  public Elevator() {
    elevatorMotorOne = SparkMax.createDefaultCANSparkMax(4);
    elevatorMotorTwo = SparkMax.createDefaultCANSparkMax(5);
    //makes elevator motors spin
  }
  public void elevatorMotorOneMovement(){
    elevatorMotorOne.set(0);
  }
  public void elevatorMotorTwoMovement(){
    elevatorMotorTwo.set(0);
  }
   //stops the elevator motors
  public void stopelevatorMotorOne(){
    elevatorMotorOne.stopMotor();
  }
  public void stopelevatorMotorTwo(){
    elevatorMotorTwo.stopMotor();
  }




  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

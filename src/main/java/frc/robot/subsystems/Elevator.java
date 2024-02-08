// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.extension.SparkMax;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  CANSparkMax elevatorMotorOne;
  CANSparkMax elevatorMotorTwo;

  // Sets Elevator Motor Object ID's
  public Elevator() {
    elevatorMotorOne = SparkMax.createDefaultCANSparkMax(2);
    elevatorMotorTwo = SparkMax.createDefaultCANSparkMax(3);
  }

  // Human control of elevator motors
  public void elevatorMotorOneMovement() {
    elevatorMotorOne.set(0);
  }

  public void elevatorMotorTwoMovement() {
    elevatorMotorTwo.set(0);

  }

  public void elevatorMotorsMovements(){
    elevatorMotorTwo.follow(elevatorMotorOne);
    elevatorMotorOne.set(0);
  }

  // stops the elevator motors
  public void stopElevatorMotorOne() {
    elevatorMotorOne.stopMotor();
  }

  public void stopElevatorMotorTwo() {
    elevatorMotorTwo.stopMotor();
  }

  public void stopElevatorMotors() {
    elevatorMotorOne.stopMotor();
    elevatorMotorTwo.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

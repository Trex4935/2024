// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.extension.SparkMax;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  CANSparkMax climberMotorOne;
  CANSparkMax climberMotorTwo;

  // Sets Climber Motor Object ID's
  public Climber() {
    climberMotorOne = SparkMax.createDefaultCANSparkMax(2);
    climberMotorTwo = SparkMax.createDefaultCANSparkMax(3);
  }

  // Human control of elevator motors
  public void setClimberMotorOne() {
    climberMotorOne.set(0);
  }

  public void setClimberMotorTwo() {
    climberMotorTwo.set(0);
  }

  // Makes the elevator motors run at the same time
  public void setClimberMotors() {
    climberMotorTwo.follow(climberMotorOne);
    climberMotorOne.set(0);
  }

  // stops the elevator motors
  public void stopClimberMotorOne() {
    climberMotorOne.stopMotor();
  }

  public void stopClimberMotorTwo() {
    climberMotorTwo.stopMotor();
  }

  public void stopClimberMotors() {
    climberMotorOne.stopMotor();
    climberMotorTwo.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

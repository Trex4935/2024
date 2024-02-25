// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.extension.SparkMax;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  CANSparkMax climberMotorOne, climberMotorTwo;

  // Sets Climber Motor Object ID's
  public Climber() {
    climberMotorOne = SparkMax.createDefaultCANSparkMax(2);
    climberMotorTwo = SparkMax.createDefaultCANSparkMax(3);
  }

  /** Sets the left climber motor's speed */
  public void setClimberMotorOne() {
    climberMotorOne.set(0);
  }

	/** Sets the right climber motor's speed */
  public void setClimberMotorTwo() {
    climberMotorTwo.set(0);
  }

  /** Sets both climber motors */
  public void setClimberMotors() {
    climberMotorTwo.follow(climberMotorOne);
    climberMotorOne.set(0);
  }

  /** Stops left climber motor */
  public void stopClimberMotorOne() {
    climberMotorOne.stopMotor();
  }

	/** Stops right climber motor */
  public void stopClimberMotorTwo() {
    climberMotorTwo.stopMotor();
  }

	/** Stops both climber motors */
  public void stopClimberMotors() {
    climberMotorOne.stopMotor();
    climberMotorTwo.stopMotor();
  }

		// TODO: Add any sendables we want from Climber
	public void initSendable(SendableBuilder builder) {
		
	}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

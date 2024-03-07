// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.extension.NoteState;
import frc.robot.extension.SparkMax;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  CANSparkMax climberMotorRight, climberMotorLeft;

  // Sets Climber Motor Object ID's
  public Climber() {
    climberMotorRight = SparkMax.createDefaultCANSparkMax(2);
    climberMotorLeft = SparkMax.createDefaultCANSparkMax(3);
    climberMotorLeft.setInverted(false);
    climberMotorRight.setInverted(true);

  }

  /** Sets the left climber motor's speed */
  public void setClimberMotorOne(double speed) {
    if (RobotContainer.noteLifecycle == NoteState.READYCLIMB)
    {
      climberMotorRight.set(speed);
    }
    if (RobotContainer.noteLifecycle == NoteState.CLIMB)
    {
      climberMotorRight.set(-speed);
    }
    climberMotorRight.set(speed);
  }

  /** Sets the right climber motor's speed */
  public void setClimberMotorTwo(double speed) {
    if (RobotContainer.noteLifecycle == NoteState.READYCLIMB)
    {
      climberMotorLeft.set(speed);
    }
    if (RobotContainer.noteLifecycle == NoteState.CLIMB)
    {
      climberMotorLeft.set(-speed);
    }
    climberMotorRight.set(speed);
  }

  /** Sets both climber motors */
  public void setClimberMotors(double speed) {
    climberMotorLeft.set(speed);
    climberMotorRight.set(speed);
  }

  /** Stops left climber motor */
  public void stopClimberMotorOne() {
    climberMotorRight.stopMotor();
  }

  /** Stops right climber motor */
  public void stopClimberMotorTwo() {
    climberMotorLeft.stopMotor();
  }

  /** Stops both climber motors */
  public void stopClimberMotors() {
    climberMotorRight.stopMotor();
    climberMotorLeft.stopMotor();
  }

  public void initSendable(SendableBuilder builder) {
		builder.addDoubleProperty("Left Climber Encoder Position", () -> climberMotorLeft.getEncoder().getPosition(), null);
		builder.addDoubleProperty("Right Climber Encoder Position", () -> climberMotorRight.getEncoder().getPosition(), null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

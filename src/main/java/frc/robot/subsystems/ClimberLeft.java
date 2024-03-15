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

public class ClimberLeft extends SubsystemBase {
  /** Creates a new Climber. */
  CANSparkMax climberMotorLeft;

  // Sets Climber Motor Object ID's
  public ClimberLeft() {
    climberMotorLeft = SparkMax.createDefaultCANSparkMax(3);
    climberMotorLeft.setInverted(false);
  }

  /** Sets the right climber motor's speed */
  public void setClimberMotorTwo(double speed) {
    if (RobotContainer.noteLifecycle == NoteState.READYCLIMB) {
      climberMotorLeft.set(speed * .5);
    }
    if (RobotContainer.noteLifecycle == NoteState.CLIMB) {
      climberMotorLeft.set(-speed);
    }
  }

  /** Sets both climber motors */
  public void setClimberMotors(double speed) {
    climberMotorLeft.set(speed);
  }

  /** Stops left climber motor */
  public void stopClimberMotorTwo() {
    climberMotorLeft.stopMotor();
  }

  /** Stops both climber motors */
  public void stopClimberMotors() {
    climberMotorLeft.stopMotor();
  }

  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty(
        "Left Climber Encoder Position", () -> climberMotorLeft.getEncoder().getPosition(), null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

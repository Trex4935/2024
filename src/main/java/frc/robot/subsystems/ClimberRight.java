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

public class ClimberRight extends SubsystemBase {
  /** Creates a new Climber. */
  CANSparkMax climberMotorRight;

  // Sets Climber Motor Object ID's
  public ClimberRight() {
    climberMotorRight = SparkMax.createDefaultCANSparkMax(2);
    climberMotorRight.setInverted(true);
  }

  /** Sets the left climber motor's speed */
  public void setClimberMotorOne(double speed) {
    if (RobotContainer.noteLifecycle == NoteState.READYCLIMB) {
      climberMotorRight.set(speed * .5);
    }
    if (RobotContainer.noteLifecycle == NoteState.CLIMB) {
      climberMotorRight.set(-speed);
    }
  }

  /** Sets both climber motors */
  public void setClimberMotors(double speed) {
    climberMotorRight.set(speed);
  }

  /** Stops left climber motor */
  public void stopClimberMotorOne() {
    climberMotorRight.stopMotor();
  }

  /** Stops both climber motors */
  public void stopClimberMotors() {
    climberMotorRight.stopMotor();
  }

  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty(
        "Right Climber Encoder Position", () -> climberMotorRight.getEncoder().getPosition(), null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

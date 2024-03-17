// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.extension.SparkMax;

public class Shooter extends SubsystemBase {
  // Declaring Motors
  CANSparkMax shootingMotorLeft, shootingMotorRight;
  static boolean speedState = false;

  /** Creates a new Shooter. */
  public Shooter() {

    // Creating Motor Objects
    shootingMotorLeft = SparkMax.createDefaultCANSparkMax(6);
    shootingMotorRight = SparkMax.createDefaultCANSparkMax(8);
    shootingMotorRight.setInverted(true);
  }

  /** Sets the left shooting motor's speed */
  public void setShootingMotorLeft(double speed) {
    shootingMotorLeft.set(speed);
  }

  /** Sets the right shooting motor's speed */
  public void setShootingMotorRight(double speed) {
    shootingMotorRight.set(speed);
  }

  /** makes motors spin YIPPIE!! */
  public void setShooters(double motor1Speed, double motor2Speed) {
    shootingMotorLeft.set(motor1Speed);
    shootingMotorRight.set(motor2Speed);
    if (shootingMotorLeft.get() == motor1Speed) {
      speedState = true;
    }
  }

  /** Stops left shooting motor */
  public void stopShootingMotorLeft() {
    shootingMotorLeft.stopMotor();
  }

  /** Stops right shooting motor */
  public void stopShootingMotorRight() {
    shootingMotorRight.stopMotor();
  }

  /** Stop both shooting motors */
  public void stopShootingMotors() {
    stopShootingMotorLeft();
    stopShootingMotorRight();
  }

  /** state machine for shooter motors */
  public void shooterSwitch() {
    switch (RobotContainer.noteLifecycle) {

        // Note is dropped into the amp
      case AMP:
        setShooters(0.2, 0.2);
        break;
        // Note is shot out towards speaker
      case SPEAKER:
        setShooters(0.9, 0.9);
        break;
      case EJECT:
        setShooters(0.2, 0.2);
        break;
      case SOURCE:
        setShooters(-0.25, -0.25);
        break;
      case TRAP:
        if (Pivot.pivotAtAngle) {
          setShooters(0.3, 0.3);
          break;
        }
      default:
        // turns all the motors off
        stopShootingMotors();
    }
  }

  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Left Shooter Motor Speed", () -> shootingMotorLeft.get(), null);
    builder.addDoubleProperty("Right Shooter Motor Speed", () -> shootingMotorRight.get(), null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}

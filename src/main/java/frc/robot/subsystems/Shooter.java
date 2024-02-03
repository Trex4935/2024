// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.extension.FlippedDIO;
import frc.robot.extension.SparkMax;

public class Shooter extends SubsystemBase {

  // Declaring Motors
  CANSparkMax shootingmotor1;
  CANSparkMax shootingmotor2;

  // Makes a new state for the shooter

  /** Creates a new Shooter. */
  public Shooter() {

    // Creating Motor Objects
    shootingmotor1 = SparkMax.createDefaultCANSparkMax(6);
    shootingmotor2 = SparkMax.createDefaultCANSparkMax(8);

  }

  // makes motors spin YIPPIE
  public void shooterMovement(double speed) {
    shootingmotor1.set(speed);
    shootingmotor2.set(-speed);
  }

  // stops motor 1
  public void stopShootingMotor1() {
    shootingmotor1.stopMotor();
  }

  // stops motor 2
  public void stopShootingMotor2() {
    shootingmotor2.stopMotor();
  }

  // stop all motors
  public void stopShooterMotors() {
    stopShootingMotor1();
    stopShootingMotor2();
  }

  // state machine for shooter motors
  public void shooterSwitch() {
    switch (RobotContainer.noteLifecycle) {

      // Note is moving to the amp drop position
      case AMPLOADING:
        shooterMovement(0);
        break;
      // Note is dropped into the amp
      case AMP:
        shooterMovement(0);
        break;
      // Note is shot out towards speaker
      case SPEAKER:
        shooterMovement(0);

        break;
      // Deafult Position of the Shooter angled at 180 degrees approximately
      default:
        // turns all the motors off
        stopShooterMotors();

    }

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

}

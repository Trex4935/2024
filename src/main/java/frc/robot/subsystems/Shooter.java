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

  // Declaring Inputs
  public FlippedDIO shooterSmacna;

  // Makes a new state for the shooter

  /** Creates a new Shooter. */
  public Shooter() {

    // Creating Motor Objects
    shootingmotor1 = SparkMax.createDefaultCANSparkMax(16);
    shootingmotor2 = SparkMax.createDefaultCANSparkMax(26);

    // Creating Inputs
    shooterSmacna = new FlippedDIO(0);

  }

  // makes motors spin YIPPIE
  public void shooterMovement(double speed) {
    shootingmotor1.set(speed);
    shootingmotor2.set(-speed);
  }

  // stops motors
  public void stopShootingMotor1() {
    shootingmotor1.stopMotor();
  }

  public void stopShootingMotor2() {
    shootingmotor2.stopMotor();
  }

  public void stopAllMotors() {
    stopShootingMotor1();
    stopShootingMotor2();
  }

  // state machine for shooter motors
  public void shooterSwitch() {
    switch (RobotContainer.noteLifecycle) {

      case AMPLOADING:
        shooterMovement(0);
        break;

      case AMP:
        shooterMovement(0);

        break;

      case SPEAKER:
        shooterMovement(0);

        break;

      default:
        // turns all the motors off
        stopAllMotors();

    }

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

}

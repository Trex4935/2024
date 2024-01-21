// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.extension.SparkMax;
 
public class Pivot extends SubsystemBase {
  /** Creates a new Pivot. */
   CANSparkMax pivotMotor;
  public Pivot() {
    pivotMotor = SparkMax.createDefaultCANSparkMax(17);
  }
/** makes pivot motor move */
  public void runPivotMotor(){
    pivotMotor.set(0);
    pivotMotor.getPIDController().setReference(180, CANSparkBase.ControlType.kPosition);
  }

  public void stopPivotMotor(){
    pivotMotor.stopMotor();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

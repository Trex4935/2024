// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.apriltag.jni.AprilTagJNI.Helper;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.extension.ShooterLevel;
import frc.robot.extension.SparkMax;


 
public class Pivot extends SubsystemBase {

  /** Creates a new Pivot. */
  CANSparkMax pivotMotor;

   // Initializes a duty cycle encoder
  RelativeEncoder relativeEncoder;
   // Makes a Hash Map for the Pivot State Machine
  private HashMap<String, Double> stateAngle;

  public static ShooterLevel shooterLevel;

  public static boolean pivotAtAngle;

  public Pivot() {

    pivotMotor = SparkMax.createDefaultCANSparkMax(17);
    pivotMotor = SparkMax.configPIDwithSmartMotion(pivotMotor, 0, 0, 0, 0, 0, 1, 1, 0);
    shooterLevel = ShooterLevel.Default;
    relativeEncoder = pivotMotor.getEncoder();
    pivotMotor.getPIDController().setFeedbackDevice(relativeEncoder);
    stateAngle = new HashMap<String,Double>();
    stateAngle.put("Default", 30.0);
    stateAngle.put("Amp", 60.0);
    stateAngle.put("Speaker", 90.0);
    stateAngle.put("Feed", 120.0);
    stateAngle.put("Load", 150.0);


  }

/** makes pivot motor move */
  public void runPivotMotor(){
    pivotMotor.set(0.1);
  }

  public void reversePivotMotor(){
    pivotMotor.set(-0.1);
  }

  public void stopPivotMotor(){
    pivotMotor.stopMotor();
  }

  // changes the state of the shooter
  public void changeShooterLevel(ShooterLevel desiredLevel){
    shooterLevel = desiredLevel;
    System.out.println(shooterLevel);
  }

  // returns the current state of the shooter
  public String returnShooterLevel(){
    System.out.println(shooterLevel.toString());
    return shooterLevel.toString();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // takes in a state and makes it the current one
  public Command stateSwitcher(ShooterLevel desiredLevel){
    return runOnce(
      () -> changeShooterLevel(desiredLevel)
    );
  } 

  // Shooter state machine that switches between different angles
  public void shooterStateMachine() {
    double targetAngle = stateAngle.get(returnShooterLevel());
    pivotMotor.getPIDController().setReference(targetAngle, CANSparkBase.ControlType.kSmartMotion);
    // Checks to see if the pivot angle is close to the expected angle, can be used anywhere
    pivotAtAngle = MathUtil.isNear(targetAngle, pivotMotor.getEncoder().getPosition(), 50.0);
  }
}

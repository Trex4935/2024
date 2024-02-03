// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;

import edu.wpi.first.apriltag.jni.AprilTagJNI.Helper;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.extension.FlippedDIO;
import frc.robot.extension.PivotAngle;
import frc.robot.extension.SparkMax;

public class Pivot extends SubsystemBase {
  FlippedDIO limitSwitch;
  FlippedDIO limitSwitch2;

  /** Creates a new Pivot. */
  CANSparkMax pivotMotor;

  // Initializes a duty cycle encoder
  RelativeEncoder relativeEncoder;
  // Makes a Hash Map for the Pivot State Machine
  private HashMap<String, Double> stateAngle;
  // Creates Pivot Angle and Pivot at Angle Objects
  public static PivotAngle pivotAngle;

  public static boolean pivotAtAngle;

  public Pivot() {

    pivotMotor = SparkMax.createDefaultCANSparkMax(7);
    pivotMotor = SparkMax.configPIDwithSmartMotion(pivotMotor, 0, 0, 0, 0, 0, 1, 1, 0);
    pivotAngle = PivotAngle.Default;
    relativeEncoder = pivotMotor.getEncoder();
    pivotMotor.getPIDController().setFeedbackDevice(relativeEncoder);

    limitSwitch = new FlippedDIO(1);
    stateAngle = new HashMap<String, Double>();
    stateAngle.put("Default", 30.0);
    stateAngle.put("Amp", 60.0);
    stateAngle.put("Speaker", 90.0);
    stateAngle.put("Feed", 120.0);
    stateAngle.put("Load", 150.0);

  }

  /** makes pivot motor move */
  public void runPivotMotor() {
    pivotMotor.set(0.1);
  }

  public void reversePivotMotor() {
    pivotMotor.set(-0.1);
  }

  public void stopPivotMotor() {
    pivotMotor.stopMotor();
  }

  // changes the state of the shooter
  public void changePivotAngle(PivotAngle desiredLevel) {
    pivotAngle = desiredLevel;
    // System.out.println(pivotAngle);
  }

  // returns the current state of the shooter
  public String returnPivotAngle() {
    // System.out.println(pivotAngle.toString());
    return pivotAngle.toString();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // takes in a state and makes it the current one
  public Command stateSwitcher(PivotAngle desiredLevel) {
    return runOnce(
        () -> changePivotAngle(desiredLevel));
  }

  // Shooter state machine that switches between different angles
  public void pivotStateMachine() {
    double targetAngle = stateAngle.get(returnPivotAngle());
    pivotMotor.getPIDController().setReference(targetAngle, CANSparkBase.ControlType.kSmartMotion);
    // Checks to see if the pivot angle is close to the expected angle, can be used
    // anywhere
    pivotAtAngle = MathUtil.isNear(targetAngle, pivotMotor.getEncoder().getPosition(), 50.0);
  }

  public void limitSwitchStop() {
    if (limitSwitch.get()) {

    } else if (limitSwitch2.get()) {

    } else {
    }
  }

  public Command applyRequest(Object object) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'applyRequest'");
  }
}

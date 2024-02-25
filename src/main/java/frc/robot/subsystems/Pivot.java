// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.extension.FlippedDIO;
import frc.robot.extension.PivotAngle;
import frc.robot.extension.SparkMax;

// TODO: Finish cleanup after pivot tuning
public class Pivot extends SubsystemBase {
  // Creates two new limit switches
  FlippedDIO limitSwitch, limitSwitch2;
  double zeroRead;

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
    // News up pivot motor and configs it to the PID
    pivotMotor = SparkMax.createDefaultCANSparkMax(7);
    pivotMotor = SparkMax.configPIDwithSmartMotion(pivotMotor, 0.0005, 0, 0, 0, 0, 0.1, 0.1, 5);

    // Sets the pivot state machine
    pivotAngle = PivotAngle.Default;

    // News up the relative encoder and configs it to the PID
    relativeEncoder = pivotMotor.getEncoder();
    pivotMotor.getPIDController().setFeedbackDevice(relativeEncoder);

    // News up the limit switches
    limitSwitch = new FlippedDIO(4);
    limitSwitch2 = new FlippedDIO(5);

    // News up the Hash Map and adds the pivot values to it
    stateAngle = new HashMap<String, Double>();
    stateAngle.put("Default", 30.0);
    stateAngle.put("Amp", 60.0);
    stateAngle.put("Speaker", 90.0);
    stateAngle.put("Feed", 120.0);
    stateAngle.put("Load", 150.0);

  }

  /** Sets the pivot motor's speed */
  public void setPivotMotor(double speed) {
    pivotMotor.set(speed);
    testLimitSwitch();

  }

  public void testLimitSwitch() {
    if (pivotMotor.get() > 0) {
      System.out.println("Forward Pivot");
      if (limitSwitch.get()) {
        pivotMotor.set(0);
      }
    }
    if (pivotMotor.get() < 0) {
      System.out.println("Reverse Pivot");
      if (limitSwitch2.get()) {
        pivotMotor.set(0);
        zeroRead = relativeEncoder.getPosition();
      }
    }

  }

  // Manual movement for the PID
  public void setPivotPosition(String desiredPosition) {
    double targetAngle = stateAngle.get(desiredPosition);
    double adjustedAngle = targetAngle + zeroRead;
    pivotMotor.getPIDController().setReference(adjustedAngle, CANSparkBase.ControlType.kPosition);
    System.out.println(adjustedAngle);
    testLimitSwitch();
  }

  /** Stops the pivot motor */
  public void stopPivotMotor() {
    pivotMotor.stopMotor();
  }

  /** Changes the state of the shooter */
  public void changePivotAngle(PivotAngle desiredLevel) {
    pivotAngle = desiredLevel;
    // System.out.println(pivotAngle);
  }

  // returns the current state of the shooter
  public String returnPivotAngle(PivotAngle desiredAngle) {
    desiredAngle = pivotAngle;
    return pivotAngle.toString();
  }

  // takes in a state and makes it the current one
  public Command stateSwitcher(PivotAngle desiredLevel) {
    return runOnce(
        () -> changePivotAngle(desiredLevel));
  }

  // Shooter state machine that switches between different angles
  public void pivotSwitch(PivotAngle desiredAngle) {
    switch (RobotContainer.noteLifecycle) {

      // Note is moving to the amp drop position
      case AMPLOADING:
        setPivotPosition("");
        break;
      // Note is dropped into the amp
      case AMP:
        setPivotPosition("Amp");
        ;
        break;
      // Note is shot out towards speaker
      case SPEAKER:
        setPivotPosition("Speaker");
        ;
        break;
      case EJECT:
        setPivotPosition("");
        break;
      // Default Position of the Shooter angled at 180 degrees approximately
      default:
        // turns all the motors off
        stopPivotMotor();
    }

  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Pivot Encoder Position", () -> relativeEncoder.getPosition(), null);
    builder.addDoubleProperty("Pivot Encoder Velocity", () -> relativeEncoder.getVelocity(), null);
    builder.addBooleanProperty("Limit Switch 2", () -> limitSwitch2.get(), null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

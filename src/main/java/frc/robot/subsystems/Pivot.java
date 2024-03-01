// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkLimitSwitch;


import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.extension.FlippedDIO;
import frc.robot.extension.PivotAngle;
import frc.robot.extension.SparkMax;
import frc.robot.extension.Helper;

// TODO: Finish cleanup after pivot tuning
public class Pivot extends SubsystemBase {
  // Creates two new limit switches
  double zeroRead;
  SparkLimitSwitch batteryLimitSwitch, forceFieldLimitSwitch;
  boolean currentLimitSwitch = false;
  boolean previousLimitSwitch = true;

  /** Creates a new Pivot. */
  CANSparkMax pivotMotor;

  // Initializes a duty cycle encoder
  RelativeEncoder relativeEncoder;
	// Initializes the pivot motor's PID
	SparkPIDController pivotPID;

  // Makes a Hash Map for the Pivot State Machine
  private HashMap<String, Double> stateAngle;
  // Creates Pivot Angle and Pivot at Angle Objects
  public static PivotAngle pivotAngle;
  public static boolean pivotAtAngle;

  public Pivot() {
    // News up pivot motor and configs it to the PID
    pivotMotor = SparkMax.createDefaultCANSparkMax(7);
		pivotPID = pivotMotor.getPIDController();
		SparkMax.configPIDforPositionControl(pivotPID, 0.1, 0, 0, 0, 0, -0.3, 0.3);

    // Sets the pivot state machine
    pivotAngle = PivotAngle.Default;

    // News up the relative encoder and configs it to the PID
    relativeEncoder = pivotMotor.getEncoder();
    pivotPID.setFeedbackDevice(relativeEncoder);

    // News up the limit switches
    batteryLimitSwitch = pivotMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
    forceFieldLimitSwitch = pivotMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);

    // News up the Hash Map and adds the pivot values to it
    stateAngle = new HashMap<String, Double>();
    stateAngle.put("Default", -40.0);
    stateAngle.put("Amp", 0.0);
    stateAngle.put("Speaker", -25.0);
    stateAngle.put("Feed", 15.0);
    stateAngle.put("Load", 0.0);

  }

  // Sets motor speed if limit switches aren't pressed
  public void setPivotMotor(double speed) {
    if(testLimitSwitch(speed))
    {
      pivotMotor.set(0);
    }
    else
    {
      pivotMotor.set(speed);
    }
  }

  // Checks to see if the speed is at our target speed with limit switch??
  public boolean testLimitSwitch(double speed) {
    if (speed < 0 && batteryLimitSwitch.isPressed()) {
      // System.out.println("Forward Pivot");
      zeroRead = relativeEncoder.getPosition();
      return true;
    }
    if (speed > 0 && forceFieldLimitSwitch.isPressed()) {
      // System.out.println("Reverse Pivot");
      return true;
    }
    currentLimitSwitch = batteryLimitSwitch.isPressed();
    if (Helper.detectFallingRisingEdge(previousLimitSwitch, currentLimitSwitch, true)){
      relativeEncoder.setPosition(-40);
    }
    previousLimitSwitch = currentLimitSwitch;
    return false;
  }

  // Manual movement for the PID
  public void setPivotPosition(String desiredPosition) {
    double targetAngle = stateAngle.get(desiredPosition);
    double adjustedAngle = targetAngle + zeroRead;
    pivotPID.setReference(adjustedAngle, CANSparkBase.ControlType.kPosition);
    System.out.println("TA: " + targetAngle);
    // testLimitSwitch();
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
        setPivotPosition("Default");
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
    builder.addBooleanProperty("Limit Switch 2", () -> forceFieldLimitSwitch.isPressed(), null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

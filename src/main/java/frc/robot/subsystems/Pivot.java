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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.extension.SparkMax;
import frc.robot.extension.Helper;


public class Pivot extends SubsystemBase {
  // Creates two new limit switches
  SparkLimitSwitch batteryLimitSwitch, forceFieldLimitSwitch;
  boolean currentLimitSwitch = false;
  boolean previousLimitSwitch = true;

  /** Creates a new Pivot. */
  CANSparkMax pivotMotor;

  // Initializes a duty cycle encoder
  RelativeEncoder relativeEncoder;

  // Initializes the pivot motor's PID
  SparkPIDController pivotPID;

  // offset
  double offsetAngle = 0.0;

  // Makes a Hash Map for the Pivot State Machine
  private HashMap<String, Double> stateAngle;
  // Creates Pivot at Angle Object
  public static boolean pivotAtAngle;

  public Pivot() {
    // News up pivot motor and configs it to the PID
    pivotMotor = SparkMax.createDefaultCANSparkMax(7);
    pivotPID = pivotMotor.getPIDController();
    SparkMax.configPIDforPositionControl(pivotPID, 0.1, 0, 0, 0, 0, -0.3, 0.3);

    // News up the relative encoder and configs it to the PID
    relativeEncoder = pivotMotor.getEncoder();
    pivotPID.setFeedbackDevice(relativeEncoder);

    // News up the limit switches
    batteryLimitSwitch = pivotMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
    forceFieldLimitSwitch = pivotMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);

    // News up the Hash Map and adds the pivot values to
    stateAngle = new HashMap<String, Double>();
    stateAngle.put("Default", -55.0);
    stateAngle.put("Amp", -36.0);
    stateAngle.put("Speaker", -20.0); // -25
    stateAngle.put("Source", -36.0);
    stateAngle.put("Climb", 0.0);
    stateAngle.put("Trap", 0.0);

  }

  // Sets motor speed if limit switches aren't pressed
  public void setPivotMotor(double speed) {
    if (testLimitSwitch(speed)) {
      pivotMotor.set(0);
    } else {
      pivotMotor.set(speed);
    }
  }

  // Checks to see if the speed is at our target speed with limit switch??
  public boolean testLimitSwitch(double speed) {
    if ((speed < 0 && batteryLimitSwitch.isPressed()) || (speed > 0 && forceFieldLimitSwitch.isPressed())) {
    	return true;
    }

    currentLimitSwitch = batteryLimitSwitch.isPressed();
    if (Helper.detectFallingRisingEdge(previousLimitSwitch, currentLimitSwitch, true)) {
      relativeEncoder.setPosition(-55);
    }
    previousLimitSwitch = currentLimitSwitch;
    return false;
  }

  // Manual movement for the PID
  public void setPivotPosition(String desiredPosition) {
    double targetAngle = stateAngle.get(desiredPosition) + offsetAngle;
    pivotPID.setReference(targetAngle, CANSparkBase.ControlType.kPosition);
    pivotAtAngle = MathUtil.isNear(targetAngle, relativeEncoder.getPosition(), 0.4);
  }

  public void manualPivotForward() {
    offsetAngle = offsetAngle + 0.5;
  }

  public void manualPivotBackward() {
    offsetAngle = offsetAngle - 0.5;
  }

  public void resetPivotOffset() {
    offsetAngle = 0;

  }

  /** Stops the pivot motor */
  public void stopPivotMotor() {
    pivotMotor.stopMotor();
  }

  // Shooter state machine that switches between different angles
  public void pivotSwitch() {
    switch (RobotContainer.noteLifecycle) {

      // Note is dropped into the amp
      case AMP:
        setPivotPosition("Amp");
        break;
      // Note is shot out towards speaker
      case SPEAKER:
        setPivotPosition("Speaker");
        break;
      case SOURCE:
        setPivotPosition("Source");
        break;
      case TRAP:
        setPivotPosition("Trap");
        break;
      case READYCLIMB:
        setPivotPosition("Default");
        break;
      case CLIMB:
        setPivotPosition("Climb");
        break;
      default:
        setPivotPosition("Default");
        resetPivotOffset();
    }

  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Pivot Encoder Position", () -> relativeEncoder.getPosition(), null);
    builder.addDoubleProperty("Pivot Encoder Velocity", () -> relativeEncoder.getVelocity(), null);
    builder.addBooleanProperty("Force Field Limit Switch", () -> forceFieldLimitSwitch.isPressed(), null);
    builder.addBooleanProperty("Battery Limit Switch", () -> batteryLimitSwitch.isPressed(), null);
    builder.addBooleanProperty("Pivot At Angle", () -> pivotAtAngle, null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

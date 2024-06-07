// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.extension.SparkMax;
import java.util.HashMap;

public class Pivot extends SubsystemBase {
  // Creates a Pigeon IMU
  Pigeon2 pigeon2 = new Pigeon2(8);
  // WPI_PigeonIMU gyro = new WPI_PigeonIMU(6);
  // Creates two new limit switches
  SparkLimitSwitch batteryLimitSwitch, forceFieldLimitSwitch;
  boolean currentLimitSwitch = false;
  boolean previousLimitSwitch = true;

  /** Creates a new Pivot. */
  CANSparkMax pivotMotor;

  // Creates a new PID
  private PIDController PID;
  // offset
  double offsetAngle = 0.0;

  // Makes a Hash Map for the Pivot State Machine
  private HashMap<String, Double> stateAngle;
  // Creates Pivot at Angle Object
  public static boolean pivotAtAngle;

  private final Pigeon2 pidgey;

  public Pivot() {
    // News up
    PID = new PIDController(0.015, 0, 0.0);
    // News up Pigeon IMU
    pidgey = new Pigeon2(18);
    // News up pivot motor and configs it to the PID
    pivotMotor = SparkMax.createDefaultCANSparkMax(7);
    pivotMotor.setInverted(true);
    // News up the limit switches
    batteryLimitSwitch = pivotMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
    forceFieldLimitSwitch = pivotMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);

    // News up the Hash Map and adds the pivot values to
    stateAngle = new HashMap<String, Double>();
    stateAngle.put("Default", 170.0);
    stateAngle.put("Amp", 145.0);
    stateAngle.put("Speaker", 60.0); // -25
    stateAngle.put("Source", 63.0);
    stateAngle.put("Climb", 100.0);
    stateAngle.put("Trap", 100.0);
    stateAngle.put("Intake", 88.0);
    stateAngle.put("Control", 90.0);
    stateAngle.put("SpeakerFront", 130.0);
  }

  // motor speed if limit switches aren't pressed
  public void setPivotMotor(double speed) {
    pivotMotor.set(speed);
  }

  // Checks to see if the speed is at our target speed with limit switch??
  public void testLimitSwitch() {
    currentLimitSwitch = batteryLimitSwitch.isPressed();
    for (int index = 0; index % 10 == 0; index++) {
      if (currentLimitSwitch) {
        // relativeEncoder.setPosition(-50);
      }
    }
  }

  // Manual movement for the PID
  public void setPivotPosition(String desiredPosition) {
    // double targetAngle = stateAngle.get(desiredPosition) + offsetAngle;
    double targetAngle = stateAngle.get(desiredPosition);
    double currentRoll = pidgey.getRoll().getValueAsDouble();
    if (currentRoll < -90) {
      currentRoll = 181;
    } else if (currentRoll < 0) {
      currentRoll = 0;
    }
    pivotMotor.set(PID.calculate(currentRoll, targetAngle));
    // pivotPID.setReference(targetAngle, CANSparkBase.ControlType.kPosition);
    pivotAtAngle = MathUtil.isNear(targetAngle, pidgey.getRoll().getValueAsDouble(), 1.5);
    testLimitSwitch();
  }

  // method that reads the angle of

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
      case SPEAKERFRONT:
        setPivotPosition("SpeakerFront");
        break;
      case SOURCE:
        setPivotPosition("Source");
        break;
      case GROUNDINTAKE:
        setPivotPosition("Intake");
        break;
      case CONTROL:
        setPivotPosition("Control");
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
    // builder.addDoubleProperty("Pivot Encoder Position", () -> relativeEncoder.getPosition(),
    // null);
    // builder.addDoubleProperty("Pivot Encoder Velocity", () -> relativeEncoder.getVelocity(),
    // null);
    builder.addBooleanProperty(
        "Force Field Limit Switch", () -> forceFieldLimitSwitch.isPressed(), null);
    builder.addBooleanProperty("Battery Limit Switch", () -> batteryLimitSwitch.isPressed(), null);
    // builder.addDoubleProperty("Pivot Encoder Position", () ->
    // relativeEncoder.getPosition(), null);
    // builder.addDoubleProperty("Pivot Encoder Velocity", () ->
    // relativeEncoder.getVelocity(), null);
    // builder.addBooleanProperty("Force Field Limit Switch", () ->
    // forceFieldLimitSwitch.isPressed(), null);
    // builder.addBooleanProperty("Battery Limit Switch", () ->
    // batteryLimitSwitch.isPressed(), null);
    builder.addBooleanProperty("Pivot At Angle", () -> pivotAtAngle, null);
    // builder.addStringProperty("Pigeon IMU", () -> pidgey.getYaw().toString(),
    // null);
    builder.addDoubleProperty("Pigeon IMU Roll", () -> pidgey.getRoll().getValueAsDouble(), null);
    // builder.addStringProperty("Pigeon IMU Pitch", () ->
    // pidgey.getPitch().toString(), null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

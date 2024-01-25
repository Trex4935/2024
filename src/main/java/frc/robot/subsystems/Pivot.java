// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.extension.ShooterLevel;
import frc.robot.extension.SparkMax;
 
public class Pivot extends SubsystemBase {

  /** Creates a new Pivot. */
   CANSparkMax pivotMotor;
   PIDController pid;

   // Initializes a duty cycle encoder
   DutyCycleEncoder encoder;
   
   private HashMap<String, Double> stateAngle;
   
   
    public static ShooterLevel shooterLevel;
  public Pivot() {
    pivotMotor = SparkMax.createDefaultCANSparkMax(17);
    shooterLevel = ShooterLevel.DEFAULT;
    encoder = new DutyCycleEncoder(0);
    pid = new PIDController(0, 0 ,0);

    stateAngle = new HashMap<String,Double>();
    stateAngle.put("Default", 30.0);
    stateAngle.put("Amp", 60.0);
    stateAngle.put("Speaker", 90.0);
    stateAngle.put("Feed", 120.0);
    stateAngle.put("Load", 150.0);


  }

/** makes pivot motor move */
  public void runPivotMotor(){
    pivotMotor.set(0);
    pivotMotor.getPIDController().setReference(180, CANSparkBase.ControlType.kPosition);
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

  public void netTbls(SendableBuilder builder) {
    builder.addStringProperty("Angle", this::returnShooterLevel, null);
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
    pivotMotor.set(pid.calculate(encoder.getDistance(), (stateAngle.get(returnShooterLevel()))));
    
    }
}

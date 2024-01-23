// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.extension.ShooterLevel;
import frc.robot.extension.SparkMax;
import frc.robot.extension.ShooterLevel;
 
public class Pivot extends SubsystemBase {

  /** Creates a new Pivot. */
   CANSparkMax pivotMotor;

   // Initializes a duty cycle encoder on DIO pins 0
   DutyCycleEncoder encoder;
   
   
   
    public static ShooterLevel shooterLevel;
  public Pivot() {
    pivotMotor = SparkMax.createDefaultCANSparkMax(17);
    shooterLevel = ShooterLevel.DEFAULT;
    encoder = new DutyCycleEncoder(0);
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
    switch (shooterLevel)
    {
      // Angled towards the Human Player for loading 
      case LOAD:
        encoder.setDutyCycleRange(1.0/1024.0, 1023.0/1024.0);
        encoder.setDistancePerRotation(0.0);
        System.out.println("Load");
        break;
      case FEED:
        encoder.setDutyCycleRange(1.0/1024.0, 1023.0/1024.0);
        encoder.setDistancePerRotation(30.0);
        System.out.println("Feed");
        break;
      // Lays shooter flat, deafult position. 
      case DEFAULT:
        encoder.setDutyCycleRange(1.0/1024.0, 1023.0/1024.0);
        encoder.setDistancePerRotation(60.0);
        System.out.println("Default");
        break;
      // Angles shooter upwards
      case AMP:
        System.out.println("Amp");
         encoder.setDutyCycleRange(1.0/1024.0, 1023.0/1024.0);
        encoder.setDistancePerRotation(90.0);
        break;
      // Less steeper then Amp but more steeper then deafult, used for shooting
      case SPEAKER:
        System.out.println("Speaker");
        encoder.setDutyCycleRange(1.0/1024.0, 1023.0/1024.0);
        encoder.setDistancePerRotation(120.0);
        break;
  
      default:
      System.out.println("Deafult");
    }
    }
}

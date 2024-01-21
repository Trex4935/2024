// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.extension.SparkMax;
import frc.robot.extension.ShooterLevel;

public class Shooter extends SubsystemBase {

   //Declaring Motors
  CANSparkMax shootingmotor1;
  CANSparkMax shootingmotor2;
  CANSparkMax magazinemotor;
  CANSparkMax pivotMotor;
 
  // Makes a new state for the shooter
  public static ShooterLevel shooterLevel;


  /** Creates a new Shooter. */
  public Shooter() {

    //Creating Motor Objects
    shootingmotor1 = SparkMax.createDefaultCANSparkMax(16);
    shootingmotor2 = SparkMax.createDefaultCANSparkMax(26);
    magazinemotor = SparkMax.createDefaultCANSparkMax(36);
    pivotMotor = SparkMax.createDefaultCANSparkMax(17);
   

   shooterLevel = ShooterLevel.DEFAULT;
}
 //makes motors spin YIPPIE
  public void shooterMovement(){
    shootingmotor1.set(-0.8);
    shootingmotor2.set(0.8);
  }
//makes magazine motor spin
   public void magazineMotorMovement(){
    magazinemotor.set(0.5);
  }
//makes pivot motor move
  public void pivotMotor(){
    pivotMotor.set(0);
    pivotMotor.getPIDController().setReference(180, CANSparkBase.ControlType.kPosition);
  }
//stops motors
  public void stopShootingMotor1(){
    shootingmotor1.stopMotor();
  }
  public void stopShootingMotor2(){
    shootingmotor2.stopMotor();
  }
  public void stopMagazineMotor(){
    magazinemotor.stopMotor();
  }
  public void stopPivotMotor(){
    pivotMotor.stopMotor();
  }
  public void stopAllMotors(){
   stopShootingMotor1();
   stopShootingMotor2();
   stopMagazineMotor();
   stopPivotMotor();
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
      System.out.println("Load");
      break;
    case FEED:
      System.out.println("Feed");
      break;
    // Lays shooter flat, deafult position. 
    case DEFAULT:
      System.out.println("Deafult");
      break;
    // Angles shooter upwards
    case AMP:
      System.out.println("Amp");
      break;
    // Less steeper then Amp but more steeper then deafult, used for shooting
    case SPEAKER:
      System.out.println("Speaker");
      break;

    default:
    System.out.println("Deafult");
  }
  }
}

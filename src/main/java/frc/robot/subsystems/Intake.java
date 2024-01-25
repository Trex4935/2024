// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  // DoubleSolenoid doublesolenoid;
  Solenoid solenoid;
  /** Creates a new IntakeLift. */
  public Intake() {
    // doublesolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH,33, 44);
    solenoid = new Solenoid(PneumaticsModuleType.REVPH, 22);

  }
  public void switchIntake() {
    solenoid.toggle();
  }


  public boolean getIntakeState() {
    return solenoid.get();
  }



  public void initSendable(SendableBuilder builder){
    builder.addBooleanProperty("Intake Dropped", this::getIntakeState, null);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.extension.NoteState;

import frc.robot.extension.SparkMax;

public class Intake extends SubsystemBase {
  // DoubleSolenoid doublesolenoid;
  Solenoid solenoid;

  PowerDistribution PDH;

  private final Compressor m_compressor = new Compressor(PneumaticsModuleType.CTREPCM);
  NoteState intakeState;
 
  /** Creates a new IntakeLift. */
  public Intake() {
    // doublesolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH,33, 44);
    solenoid = new Solenoid(PneumaticsModuleType.REVPH, 22);

    PDH = new PowerDistribution(1, ModuleType.kRev);

    intakeState = NoteState.FIELD;
  }
  public void switchIntakeOn() {
    solenoid.set(true);

  }
  public void switchIntakeOff() {
    solenoid.set(false);
  }
  public void PDH_on(){
    PDH.setSwitchableChannel(true);
  }
  public void PDH_off(){
    PDH.setSwitchableChannel(false);
  }

  public boolean getIntakeState() {
    return solenoid.get();
  }
  
  public double sensePSI (){
    return m_compressor.getPressure();
  }
public boolean OnOff_Compressor (){

return m_compressor.isEnabled();
}
  

  public void initSendable(SendableBuilder builder){
    builder.addBooleanProperty("Intake Dropped", this::getIntakeState, null);
  }
  @Override
  public void periodic() {
    m_compressor.isEnabled();
    // This method will be called once per scheduler run
  }

  public void intakeSwitch(){
    switch (intakeState) {
      case INTAKE:
      // Turns Solenoid On
        switchIntakeOn();
        break;
      case GRABBED:
        switchIntakeOn();
        break;
      default:
      // Turns Solenoid Off
        switchIntakeOff();
    }
  }
}

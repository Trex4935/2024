// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.extension.NoteState;

public class Intake extends SubsystemBase {
  // DoubleSolenoid doublesolenoid;

  // Makes a New Solenoid
  Solenoid solenoid;

  // Makes a new compressor
  private final Compressor m_compressor;

  /** Creates a new IntakeLift. */
  public Intake() {
    // doublesolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH,33, 44);

    // news up the solenoid and compressor
    solenoid = new Solenoid(14, PneumaticsModuleType.CTREPCM, 2);

    // Creates Compressor Object
    m_compressor = new Compressor(14, PneumaticsModuleType.CTREPCM);
    m_compressor.enableDigital();

  }

  // Turns on the solenoid, brings intake up
  public void switchIntakeOn() {
    solenoid.set(true);
    System.out.println("Intake on");

  }

  // Turns off the solenoid, brings intake down
  public void switchIntakeOff() {
    solenoid.set(false);
    System.out.println("Intake off");

  }

  // Returns the current state of the solenoid-Used for Sendables
  public boolean getIntakeState() {
    return solenoid.get();
  }

  // puts the current state of the intake on the network table
  public void initSendable(SendableBuilder builder) {
    builder.addBooleanProperty("Intake Dropped", this::getIntakeState, null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // Note Life Cycle state machine to control the intake
  public void intakeSwitch() {
    switch (RobotContainer.noteLifecycle) {
      case FIELD:
        switchIntakeOn();
        break;
      case GROUNDINTAKE:
        // Turns Solenoid On when we want to intake a note
        switchIntakeOff();
        break;
      case GRABBED:
        System.out.println("Grabbed");
        // Keeps solenoid on when note is inside robot just to be sure
        switchIntakeOff();
        break;
      case CONTROL:
        System.out.println("Control");
        // Keeps solenoid on when note is inside robot just to be sure
        switchIntakeOff();
        break;
      default:
        // Turns Solenoid Off in all other cases
        switchIntakeOn();
    }
  }
}

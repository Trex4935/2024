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

public class DustPan extends SubsystemBase {

  // Makes a New Solenoid
  Solenoid solenoid;

  // Makes a new compressor
  private final Compressor m_compressor;

  /** Creates a new DustPan. */
  public DustPan() {
    // news up the solenoid and compressor
    solenoid = new Solenoid(14, PneumaticsModuleType.CTREPCM, 5);

    // Creates Compressor Object
    m_compressor = new Compressor(14, PneumaticsModuleType.CTREPCM);
    m_compressor.enableDigital();
  }

  /** Turns on the solenoid, brings dustpan up */
  public void dustPanUp() {
    solenoid.set(true);
  }

  /** Turns off the solenoid, brings dustpan down */
  public void dustPanDown() {
    solenoid.set(false);
  }

  /** Returns the current state of the solenoid-Used for Sendables */
  public boolean getDustPanState() {
    return solenoid.get();
  }

  /** Note Life Cycle state machine to control the intake */
  public void intakeSwitch() {
    switch (RobotContainer.noteLifecycle) {
      case FIELD:
        dustPanUp();
        break;

      case GROUNDINTAKE:
        // Turns Solenoid on when we want to intake a note
        dustPanDown();
        break;

      case GRABBED:
        // Keeps solenoid on when note is inside robot just to be sure
        dustPanDown();
        break;

      case CONTROL:
        // Keeps solenoid on when note is inside robot just to be sure
        dustPanUp();
        break;

      default:
        // Turns Solenoid off in all other cases
        dustPanUp();
    }
  }

  @Override
  /** Puts the current state of the intake onto NetworkTables */
  public void initSendable(SendableBuilder builder) {
    builder.addBooleanProperty("DustPan Up?", this::getDustPanState, null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

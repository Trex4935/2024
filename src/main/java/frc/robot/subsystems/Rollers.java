package frc.robot.subsystems;

import java.util.EnumSet;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.extension.SparkMax;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.extension.FlippedDIO;
import frc.robot.extension.NoteState;

public class Rollers extends SubsystemBase {
  CANSparkMax lowMagazine;
  CANSparkMax highMagazine;
  NoteState rollerState;

  public FlippedDIO intakeSmacna;
  public FlippedDIO magazineSmacna;
  public FlippedDIO magneticFlap;
  public FlippedDIO shooterSmacna;

  boolean previousValue;
  boolean currentValue;

  public Rollers() {
    // random id's and creating motor objects

    lowMagazine = SparkMax.createDefaultCANSparkMax(4);
    highMagazine = SparkMax.createDefaultCANSparkMax(5);

    // Sensor Objects
    intakeSmacna = new FlippedDIO(0);
    magazineSmacna = new FlippedDIO(1);
    magneticFlap = new FlippedDIO(2);
    shooterSmacna = new FlippedDIO(3);

  }

  public void onLowMagazine(double speed) {
    lowMagazine.set(speed);
  }

  public void onHighMagazine(double speed) {
    highMagazine.set(speed);
  }

  public void stopLowMagazine() {
    lowMagazine.stopMotor();
  }

  public void stopHighMagazine() {
    highMagazine.stopMotor();
  }

  @Override
  public void periodic() {
    //

  }

  // state machine for rollers
  public void rollerStateMachine() {
    switch (RobotContainer.noteLifecycle) {
      // ground intake state, turns low roller on
      case GROUNDINTAKE:
        onLowMagazine(0.1);
        // intake sensor detects leading edge of note -> Grabbed state
        if (intakeSmacna.get()) {
          RobotContainer.noteLifecycle = NoteState.GRABBED;
        }

        break;

      // Grabbed state, turns the low roller on
      case GRABBED:
        onLowMagazine(0.1);
        onHighMagazine(0.1);
        // intake sensor detects back edge of the note -> Control state
        if (!intakeSmacna.get()) {
          RobotContainer.noteLifecycle = NoteState.CONTROL;
        }
        break;
      // control state, turns low roller on
      case CONTROL:
        onLowMagazine(0.1);
        // magazine sensor detects leading edge of note -> Storage state
        if (magazineSmacna.get()) {
          RobotContainer.noteLifecycle = NoteState.STORAGE;
        }

        break;

      case STORAGE:
        // Turns low roller off; storage state
        stopLowMagazine();
        stopHighMagazine();
        break;
      case AMPLOADING:
        // Turns low and high rollers on; amploading state
        onHighMagazine(0.1);
        onLowMagazine(0.1);
        // If the magnetic flap moves away from magnet -> AMP state
        if (!magneticFlap.get()) {
          RobotContainer.noteLifecycle = NoteState.AMP;
        }
        break;
      case SPEAKER:
        // speaker state, turns both high and low rollers on
        onLowMagazine(0.1);
        onHighMagazine(0.1);
        currentValue = shooterSmacna.get();
        if (previousValue != currentValue) {
          // if the previous value is true set note state to field
          if (previousValue) {
            RobotContainer.noteLifecycle = NoteState.FIELD;
          }
        }
        // set previousvalue to the current one
        previousValue = currentValue;
        break;
      // amp state; turns both high and low rollers on
      case AMP:
        onHighMagazine(-0.1);
        // set currentValue to current magnetic flap value
        currentValue = magneticFlap.get();
        // check if previous magnetic value does not equal current value
        if (previousValue != currentValue) {
          // if the previous value is true set note state to field
          if (previousValue) {
            RobotContainer.noteLifecycle = NoteState.FIELD;
          }
        }
        // set previousvalue to the current one
        previousValue = currentValue;
        break;
      // eject state, turns high and low rollers on
      case EJECT:

        onHighMagazine(0);
        onLowMagazine(0);
        break;

      default:
        // Turns magazines Off (default state of the rollers)
        stopHighMagazine();
        stopLowMagazine();

    }

  }
}

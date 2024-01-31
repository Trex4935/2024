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
  CANSparkMax lowmagazine;
  CANSparkMax highmagazine;

  CANSparkMax magazinemotor;
  public FlippedDIO intakeSmacna;
  public FlippedDIO magazineSmacna;
  public FlippedDIO magneticFlap;

  boolean previousValue;
  boolean currentValue;
  public Rollers() {
    // random id's and creating motor objects
    lowMagazine = SparkMax.createDefaultCANSparkMax(9);
    highMagazine = SparkMax.createDefaultCANSparkMax(10);
    // Sensor Objects
    intakeSmacna = new FlippedDIO(0);
    magazineSmacna = new FlippedDIO(1);
    magneticFlap = new FlippedDIO(2);

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

  public void intakeSwitch() {
    switch (RobotContainer.noteLifecycle) {

      case GROUNDINTAKE:
        onLowMagazine(0.1);
        // intake sensor detects leading edge of note -> Grabbed state
        if (intakeSmacna.get()) {
          RobotContainer.noteLifecycle = NoteState.GRABBED;
        }

        break;

      case GRABBED:
        // Turns the low roller on
        onLowMagazine(0.1);
        // intake sensor detects back edge of the note -> Control state
        if (!intakeSmacna.get()) {
          RobotContainer.noteLifecycle = NoteState.CONTROL;
        }

        break;

      case CONTROL:
        onLowMagazine(0.1);
        // magazine sensor detects leading edge of note -> Storage state
        if (magazineSmacna.get()) {
          RobotContainer.noteLifecycle = NoteState.STORAGE;
        }

        break;

      case STORAGE:
        // Turns low roller off
        stopLowMagazine();
        break;
      case AMPLOADING:
        // Turns low roller on
        onHighMagazine(0.1);
        onLowMagazine(0.1);
        // If the magnetic flap moes away from magnet -> Speaker state
        if (magneticFlap.get()) {
          RobotContainer.noteLifecycle = NoteState.AMP;
        }
        break;
      case SPEAKER:
        onLowMagazine(0.1);
        onHighMagazine(0.1);
        break;
      case AMP:
        onLowMagazine(-0.1);
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
      case EJECT:

        highmagazine.set(0.1);
        lowmagazine.set(0.1);

      default:
        // Turns magazines Off
        stopHighMagazine();
        stopLowMagazine();

    }
   
  }
}

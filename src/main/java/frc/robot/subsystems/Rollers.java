package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import frc.robot.extension.SparkMax;
import frc.robot.RobotContainer;
import frc.robot.extension.FlippedDIO;
import frc.robot.extension.NoteState;
import frc.robot.extension.Helper;

public class Rollers extends SubsystemBase {
  CANSparkMax lowMagazine;
  CANSparkMax highMagazine;
  NoteState rollerState;

  public FlippedDIO intakeSmacnaLeft;
  public FlippedDIO intakeSmacnaRight;
  public FlippedDIO magazineSmacna;
  public FlippedDIO magneticFlap;
  public FlippedDIO shooterSmacna;

  boolean previousValue;
  boolean currentValue;

  Timer timer;

  public Rollers() {
    // News up motor objects
    lowMagazine = SparkMax.createDefaultCANSparkMax(4);
    lowMagazine.setInverted(true);
    highMagazine = SparkMax.createDefaultCANSparkMax(5);

    // Sensor Objects
    intakeSmacnaLeft = new FlippedDIO(0);
    intakeSmacnaRight = new FlippedDIO(6);
    magazineSmacna = new FlippedDIO(1);
    magneticFlap = new FlippedDIO(2);
    shooterSmacna = new FlippedDIO(3);

    // News up a timer object
    timer = new Timer();

  }

  // Sets magazine speed
  public void onLowMagazine(double speed) {
    lowMagazine.set(speed);
  }

  public void onHighMagazine(double speed) {
    highMagazine.set(speed);
  }

  // Stops magazines
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

  // Switches the state that the rollers operate in
  public void rollerSwitch() {
    switch (rollerState) {

      // Turns low rollers on and switches state when the smacna detects note
      case GROUNDINTAKE:
        onLowMagazine(0.1);
        // intake sensor detects leading edge of note -> Grabbed state
        currentValue = intakeSmacnaLeft.get() && intakeSmacnaRight.get();
        if (Helper.detectFallingRisingEdge(previousValue, currentValue, true)) {
          RobotContainer.noteLifecycle = NoteState.GRABBED;
        }
        previousValue = currentValue;
        break;

      // Keeps low roller on
      case GRABBED:
        onLowMagazine(0.1);
        // intake sensor detects back edge of the note -> Control state
        currentValue = intakeSmacnaLeft.get() && intakeSmacnaRight.get();
        if (Helper.detectFallingRisingEdge(previousValue, currentValue, false)) {
          RobotContainer.noteLifecycle = NoteState.CONTROL;
        }
        previousValue = currentValue;
        break;

      // Keeps low roller on
      case CONTROL:
        onLowMagazine(0.1);
        // magazine sensor detects leading edge of note -> Storage state
        currentValue = magazineSmacna.get();
        if (Helper.detectFallingRisingEdge(previousValue, currentValue, true)) {
          RobotContainer.noteLifecycle = NoteState.STORAGE;
        }
        previousValue = currentValue;
        break;

      // Stops low rollers
      case STORAGE:
        stopLowMagazine();
        break;

      // Turns on both low and high rollers
      case AMPLOADING:
        // Turns low roller on
        onHighMagazine(0.1);
        onLowMagazine(0.1);
        // If the magnetic flap moes away from magnet -> Amp state
        currentValue = magazineSmacna.get();
        previousValue = currentValue;
        break;

      // Turns on both high and low rollers and returns to Field state after 5 seconds
      case SPEAKER:
        onLowMagazine(0.1);
        onHighMagazine(0.1);
        currentValue = intakeSmacnaLeft.get() && intakeSmacnaRight.get();
        if (Helper.detectFallingRisingEdge(previousValue, currentValue, false)) {
          timer.start();
          if (timer.hasElapsed(5)) {
            RobotContainer.noteLifecycle = NoteState.FIELD;
            timer.reset();
          }
        }
        previousValue = currentValue;
        break;

      // Reverses low and high rollers when maganetic flap is pushed, returns to field
      // state after 5 seconds have passed
      case AMP:
        onLowMagazine(-0.1);
        onHighMagazine(-0.1);
        currentValue = intakeSmacnaLeft.get() && intakeSmacnaRight.get();
        if (Helper.detectFallingRisingEdge(previousValue, currentValue, false)) {
          timer.start();
          if (timer.hasElapsed(5)) {
            RobotContainer.noteLifecycle = NoteState.FIELD;
            timer.reset();
          }
        }
        previousValue = currentValue;
        break;

      // Turns and keeps both high and low rollers on and turns them off again after 7
      // seconds
      case EJECT:
        highMagazine.set(0.1);
        lowMagazine.set(0.1);
        if (Helper.detectFallingRisingEdge(previousValue, currentValue, false)) {
          timer.start();
          if (timer.hasElapsed(7)) {
            RobotContainer.noteLifecycle = NoteState.FIELD;
            timer.reset();
          }
        }

        // Keeps rolllers off
      default:
        stopHighMagazine();
        stopLowMagazine();

    }
  }
}

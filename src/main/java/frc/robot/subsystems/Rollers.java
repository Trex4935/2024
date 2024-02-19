package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.extension.SparkMax;
import frc.robot.RobotContainer;
import frc.robot.extension.FlippedDIO;
import frc.robot.extension.NoteState;
import frc.robot.extension.PivotAngle;
import frc.robot.extension.Helper;

public class Rollers extends SubsystemBase {
  CANSparkMax lowMagazine;
  CANSparkMax highMagazine;

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
    intakeSmacnaRight = magazineSmacna = new FlippedDIO(1);
    magneticFlap = new FlippedDIO(2);
    shooterSmacna = new FlippedDIO(3);

    // News up a timer object
    timer = new Timer();

  }

  // Sets magazine speed
  public void onLowMagazine(double speed) {
    lowMagazine.set(speed);
  }

  public void onLowMagalzine(double speed1, double speed2) {
    lowMagazine.set(speed1);
    highMagazine.set(speed2);
  }

  public void onHighMagazine(double speed) {
    highMagazine.set(speed);
  }

  // Stops magazines
  public void stopLowMagazine() {
    lowMagazine.stopMotor();
    highMagazine.stopMotor();
  }

  public void stopHighMagazine() {
    highMagazine.stopMotor();
  }

  public void changeNoteState(NoteState noteState) {
    RobotContainer.noteLifecycle = NoteState.GROUNDINTAKE;
    // System.out.println(noteState);
  }

  @Override
  public void periodic() {
    //
    SmartDashboard.putBoolean("intakeSmacnaLeft", intakeSmacnaLeft.get());
    SmartDashboard.putBoolean("magazineSmacna", magazineSmacna.get());
    SmartDashboard.putBoolean("intakeSmacnaRight", intakeSmacnaRight.get());
    SmartDashboard.putBoolean("shooterSmacna", shooterSmacna.get());
  }

  // Switches the state that the rollers operate in
  public void rollerSwitch() {
    switch (RobotContainer.noteLifecycle) {

      // Turns low rollers on and switches state when the smacna detects note
      case GROUNDINTAKE:
        onLowMagazine(0.3);
        // intake sensor detects leading edge of note -> Grabbed state
        currentValue = intakeSmacnaLeft.get() || intakeSmacnaRight.get();
        if (Helper.detectFallingRisingEdge(previousValue, currentValue, true)) {
          RobotContainer.noteLifecycle = NoteState.GRABBED;
        }
        previousValue = currentValue;
        break;

      // Keeps low roller on
      case GRABBED:
        onLowMagazine(0.1);
        // intake sensor detects back edge of the note -> Control state
        currentValue = intakeSmacnaLeft.get() || intakeSmacnaRight.get();
        if (Helper.detectFallingRisingEdge(previousValue, currentValue, false)) {
          RobotContainer.noteLifecycle = NoteState.CONTROL;
        }
        previousValue = currentValue;
        System.out.println("GRABBED");
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
        System.out.println("CONTROL");
        break;

      // Stops low rollers
      case STORAGE:
        stopLowMagazine();
        System.out.println("STORAGE");
        break;

      // Turns on both low and high rollers
      case AMPLOADING:
        // Turns low roller on
        onHighMagazine(0.1);
        onLowMagazine(0.1);
        // If the magnetic flap moes away from magnet -> Amp state
        currentValue = magazineSmacna.get();
        previousValue = currentValue;
        System.out.println("AMPLOADING");
        break;

      // Turns on both high and low rollers and returns to Field state after 5 seconds
      case SPEAKER:
        onLowMagazine(0.1);
        onHighMagazine(0.1);
        currentValue = intakeSmacnaLeft.get() || intakeSmacnaRight.get();
        if (Helper.detectFallingRisingEdge(previousValue, currentValue, false)) {
          timer.start();
          if (timer.hasElapsed(5)) {
            RobotContainer.noteLifecycle = NoteState.FIELD;
            timer.reset();
          }
        }
        previousValue = currentValue;
        System.out.println("SPEAKER");
        break;

      // Reverses low and high rollers when maganetic flap is pushed, returns to field
      // state after 5 seconds have passed
      case AMP:
        onLowMagazine(-0.1);
        onHighMagazine(-0.1);
        currentValue = intakeSmacnaLeft.get() || intakeSmacnaRight.get();
        if (Helper.detectFallingRisingEdge(previousValue, currentValue, false)) {
          timer.start();
          if (timer.hasElapsed(5)) {
            RobotContainer.noteLifecycle = NoteState.FIELD;
            timer.reset();
          }
        }
        previousValue = currentValue;
        System.out.println("AMP");
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
        System.out.println("EJECT");

        // Keeps rolllers off
      default:
        stopHighMagazine();
        stopLowMagazine();

    }
  }
}

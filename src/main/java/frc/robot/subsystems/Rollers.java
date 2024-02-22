package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.extension.SparkMax;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.extension.FlippedDIO;
import frc.robot.extension.NoteState;
import frc.robot.extension.PivotAngle;
import frc.robot.extension.Helper;

public class Rollers extends SubsystemBase {
  CANSparkMax dustpan;
  CANSparkMax magazine;

  public FlippedDIO dustpanSmacnaLeft;
  public FlippedDIO dustpanSmacnaRight;
  public FlippedDIO magneticFlap;
  public FlippedDIO shooterSmacna;
  public FlippedDIO storageButton;

  boolean previousDustpanSmacnaState;
  boolean currentDustpanSmacnaState;
  boolean previousMagneticFlapState;
  boolean currentMagneticFlapState;

  Timer timer;

  public Rollers() {
    // News up motor objects
    dustpan = SparkMax.createDefaultCANSparkMax(4);
    dustpan.setInverted(true);
    magazine = SparkMax.createDefaultCANSparkMax(5);

    // Sensor Objects
    dustpanSmacnaLeft = new FlippedDIO(0);
    dustpanSmacnaRight = new FlippedDIO(9);
    magneticFlap = new FlippedDIO(2);
    shooterSmacna = new FlippedDIO(3);
    storageButton = new FlippedDIO(4);

    // News up a timer object
    timer = new Timer();

  }

  // sets dustpan speed
  public void setDustpan(double speed) {
    dustpan.set(speed);
  }

  // stops dustpan motor
  public void stopDustpan() {
    dustpan.stopMotor();
  }

  // sets magazine speed
  public void setMagazine(double speed) {
    magazine.set(speed);
  }

  // stops magazine motor
  public void stopMagazine() {
    magazine.stopMotor();
  }

  // sets intake speed and magazine speed.
  public void setRollers(double dustpanSpeed, double magSpeed) {
    dustpan.set(dustpanSpeed);
    magazine.set(magSpeed);
  }

  // stops rollers
  public void stopRollers() {
    stopDustpan();
    stopMagazine();
  }

  public void changeNoteState(NoteState noteState) {
    RobotContainer.noteLifecycle = NoteState.GROUNDINTAKE;
  }

  public void returnToField(NoteState noteState) {
    RobotContainer.noteLifecycle = NoteState.FIELD;
  }

  // Switches the state that the rollers operate in
  public void rollerSwitch() {
    switch (RobotContainer.noteLifecycle) {

      // Turns low rollers on and switches state when the smacna detects note
      case GROUNDINTAKE:
        setDustpan(0.9);
        // intake sensor detects leading edge of note -> Grabbed state
        currentDustpanSmacnaState = dustpanSmacnaLeft.get() || dustpanSmacnaRight.get();
        if (Helper.detectFallingRisingEdge(previousDustpanSmacnaState, currentDustpanSmacnaState, true)) {
          RobotContainer.noteLifecycle = NoteState.GRABBED;
        }
        previousDustpanSmacnaState = currentDustpanSmacnaState;
        break;

      // Keeps low roller on
      case GRABBED:
        setDustpan(0.9);
        // intake sensor detects back edge of the note -> Control state
        currentDustpanSmacnaState = dustpanSmacnaLeft.get() || dustpanSmacnaRight.get();
        if (Helper.detectFallingRisingEdge(previousDustpanSmacnaState, currentDustpanSmacnaState, false)) {
          RobotContainer.noteLifecycle = NoteState.CONTROL;
        }
        previousDustpanSmacnaState = currentDustpanSmacnaState;
        break;

      // Keeps low roller on
      case CONTROL:
        setDustpan(0.9);
        setMagazine(0.9);
        if (storageButton.equals(true)) {
          RobotContainer.noteLifecycle = NoteState.STORAGE;
        }

        break;

      // Stops low rollers
      case STORAGE:
        stopDustpan();
        stopMagazine();
        break;

      // Turns on both low and high rollers
      case AMPLOADING:
        // Turns low roller on
        setMagazine(0.1);
        setDustpan(0.1);
        // If the magnetic flap moes away from magnet -> Amp state
        currentDustpanSmacnaState = magneticFlap.get();
        previousDustpanSmacnaState = currentDustpanSmacnaState;
        break;

      // Turns on both high and low rollers and returns to Field state after 5 seconds
      case SPEAKER:
        setDustpan(0.1);
        setMagazine(0.1);
        currentDustpanSmacnaState = dustpanSmacnaLeft.get() || dustpanSmacnaRight.get();
        if (Helper.detectFallingRisingEdge(previousDustpanSmacnaState, currentDustpanSmacnaState, false)) {
          timer.start();
          if (timer.hasElapsed(5)) {
            RobotContainer.noteLifecycle = NoteState.FIELD;
            timer.reset();
          }
        }
        previousDustpanSmacnaState = currentDustpanSmacnaState;
        break;

      // Reverses low and high rollers when maganetic flap is pushed, returns to field
      // state after 5 seconds have passed
      case AMP:
        setDustpan(-0.1);
        setMagazine(-0.1);
        currentDustpanSmacnaState = dustpanSmacnaLeft.get() || dustpanSmacnaRight.get();
        if (Helper.detectFallingRisingEdge(previousDustpanSmacnaState, currentDustpanSmacnaState, false)) {
          timer.start();
          if (timer.hasElapsed(5)) {
            RobotContainer.noteLifecycle = NoteState.FIELD;
            timer.reset();
          }
        }
        previousDustpanSmacnaState = currentDustpanSmacnaState;
        break;

      // Turns and keeps both high and low rollers on and turns them off again after 7
      // seconds
      case EJECT:
        magazine.set(0.1);
        dustpan.set(0.1);
        if (Helper.detectFallingRisingEdge(previousDustpanSmacnaState, currentDustpanSmacnaState, false)) {
          timer.start();
          if (timer.hasElapsed(7)) {
            RobotContainer.noteLifecycle = NoteState.FIELD;
            timer.reset();
          }
        }

        // Keeps rollers off for magazine and dustpan
      default:
        stopMagazine();
        stopDustpan();

    }
  }

  @Override
  public void periodic() {
    //
    SmartDashboard.putBoolean("dustpanSmacnaLeft", dustpanSmacnaLeft.get());
    SmartDashboard.putBoolean("magazineSmacna", magneticFlap.get());
    SmartDashboard.putBoolean("dustpanSmacnaRight", dustpanSmacnaRight.get());
    SmartDashboard.putBoolean("shooterSmacna", shooterSmacna.get());
  }

}

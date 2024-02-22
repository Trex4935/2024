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
  CANSparkMax intake;
  CANSparkMax magazine;

  public FlippedDIO intakeSmacnaLeft;
  public FlippedDIO intakeSmacnaRight;
  public FlippedDIO magneticFlap;
  public FlippedDIO shooterSmacna;

  boolean previousIntakeSmacnaState;
  boolean currentIntakeSmacnaState;

  Timer timer;

  public Rollers() {
    // News up motor objects
    intake = SparkMax.createDefaultCANSparkMax(4);
    intake.setInverted(true);
    magazine = SparkMax.createDefaultCANSparkMax(5);

    // Sensor Objects
    intakeSmacnaLeft = new FlippedDIO(0);
    intakeSmacnaRight = new FlippedDIO(9);
    magneticFlap = new FlippedDIO(2);
    shooterSmacna = new FlippedDIO(3);

    // News up a timer object
    timer = new Timer();

  }


  public void setIntake(double speed) {
    intake.set(speed);
  }

  public void stopIntake() {
    intake.stopMotor();
  }

  public void setMagazine(double speed) {
    magazine.set(speed);
  }  

  public void stopMagazine() {
    magazine.stopMotor();
  }

    public void setRollers(double intakeSpeed, double magSpeed) {
    intake.set(intakeSpeed);
    magazine.set(magSpeed);
  }

  public void stopRollers(){
    stopIntake();
    stopMagazine();
  }

  public void changeNoteState(NoteState noteState) {
    RobotContainer.noteLifecycle = NoteState.GROUNDINTAKE;
  }

  public void returnToField(NoteState noteState) {
    RobotContainer.noteLifecycle = NoteState.FIELD;
  }

  @Override
  public void periodic() {
    //
    SmartDashboard.putBoolean("intakeSmacnaLeft", intakeSmacnaLeft.get());
    SmartDashboard.putBoolean("magazineSmacna", magneticFlap.get());
    SmartDashboard.putBoolean("intakeSmacnaRight", intakeSmacnaRight.get());
    SmartDashboard.putBoolean("shooterSmacna", shooterSmacna.get());
  }

  // Switches the state that the rollers operate in
  public void rollerSwitch() {
    switch (RobotContainer.noteLifecycle) {

      // Turns low rollers on and switches state when the smacna detects note
      case GROUNDINTAKE:
        setIntake(0.9);
        // intake sensor detects leading edge of note -> Grabbed state
        currentIntakeSmacnaState = intakeSmacnaLeft.get() || intakeSmacnaRight.get();
        if (Helper.detectFallingRisingEdge(previousIntakeSmacnaState, currentIntakeSmacnaState, true)) {
          RobotContainer.noteLifecycle = NoteState.GRABBED;
        }
        previousIntakeSmacnaState = currentIntakeSmacnaState;
        break;

      // Keeps low roller on
      case GRABBED:
        setIntake(0.9);
        // intake sensor detects back edge of the note -> Control state
        currentIntakeSmacnaState = intakeSmacnaLeft.get() || intakeSmacnaRight.get();
        if (Helper.detectFallingRisingEdge(previousIntakeSmacnaState, currentIntakeSmacnaState, false)) {
          RobotContainer.noteLifecycle = NoteState.CONTROL;
        }
        previousIntakeSmacnaState = currentIntakeSmacnaState;
        break;

      // Keeps low roller on
      case CONTROL:
        setIntake(0.9);
        // magazine sensor detects leading edge of note -> Storage state
        currentIntakeSmacnaState = magneticFlap.get();
        if (Helper.detectFallingRisingEdge(previousIntakeSmacnaState, currentIntakeSmacnaState, true)) {
          RobotContainer.noteLifecycle = NoteState.STORAGE;
        }
        previousIntakeSmacnaState = currentIntakeSmacnaState;
        break;

      // Stops low rollers
      case STORAGE:
        stopIntake();
        break;

      // Turns on both low and high rollers
      case AMPLOADING:
        // Turns low roller on
        setMagazine(0.1);
        setIntake(0.1);
        // If the magnetic flap moes away from magnet -> Amp state
        currentIntakeSmacnaState = magneticFlap.get();
        previousIntakeSmacnaState = currentIntakeSmacnaState;
        break;

      // Turns on both high and low rollers and returns to Field state after 5 seconds
      case SPEAKER:
        setIntake(0.1);
        setMagazine(0.1);
        currentIntakeSmacnaState = intakeSmacnaLeft.get() || intakeSmacnaRight.get();
        if (Helper.detectFallingRisingEdge(previousIntakeSmacnaState, currentIntakeSmacnaState, false)) {
          timer.start();
          if (timer.hasElapsed(5)) {
            RobotContainer.noteLifecycle = NoteState.FIELD;
            timer.reset();
          }
        }
        previousIntakeSmacnaState = currentIntakeSmacnaState;
        break;

      // Reverses low and high rollers when maganetic flap is pushed, returns to field
      // state after 5 seconds have passed
      case AMP:
        setIntake(-0.1);
        setMagazine(-0.1);
        currentIntakeSmacnaState = intakeSmacnaLeft.get() || intakeSmacnaRight.get();
        if (Helper.detectFallingRisingEdge(previousIntakeSmacnaState, currentIntakeSmacnaState, false)) {
          timer.start();
          if (timer.hasElapsed(5)) {
            RobotContainer.noteLifecycle = NoteState.FIELD;
            timer.reset();
          }
        }
        previousIntakeSmacnaState = currentIntakeSmacnaState;
        break;

      // Turns and keeps both high and low rollers on and turns them off again after 7
      // seconds
      case EJECT:
        magazine.set(0.1);
        intake.set(0.1);
        if (Helper.detectFallingRisingEdge(previousIntakeSmacnaState, currentIntakeSmacnaState, false)) {
          timer.start();
          if (timer.hasElapsed(7)) {
            RobotContainer.noteLifecycle = NoteState.FIELD;
            timer.reset();
          }
        }

        // Keeps rolllers off
      default:
        stopMagazine();
        stopIntake();

    }
  }
}

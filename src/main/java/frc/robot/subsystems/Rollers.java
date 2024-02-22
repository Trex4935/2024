package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.extension.SparkMax;
import frc.robot.RobotContainer;
import frc.robot.extension.FlippedDIO;
import frc.robot.extension.NoteState;
import frc.robot.extension.Helper;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Rollers extends SubsystemBase {
  // Creating mototrs; ID's shown in IDguide.md
  CANSparkMax intake;
  CANSparkMax magazine;

  // Creating Sesors; ID's shown in IDguide.md
  public FlippedDIO intakeSmacnaLeft;
  public FlippedDIO intakeSmacnaRight;
  public FlippedDIO magneticFlap;
  public FlippedDIO shooterSmacna;

  boolean previousIntakeSmacnaState;
  boolean currentIntakeSmacnaState;
  CommandGenericHID driverStationButtonPress;

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

    final CommandGenericHID driverStationButtonPress = new CommandGenericHID(1);
  }

  // moves intake motor; Sets speed
  public void setIntake(double speed) {
    intake.set(speed);
  }

  // stops intake motor
  public void stopIntake() {
    intake.stopMotor();
  }

  // moves magazine motor; Sets speed
  public void setMagazine(double speed) {
    magazine.set(speed);
  }

  // stops intake motor
  public void stopMagazine() {
    magazine.stopMotor();
  }

  // moves both motors at set speeds
  public void setRollers(double intakeSpeed, double magSpeed) {
    intake.set(intakeSpeed);
    magazine.set(magSpeed);
  }

  // stops both motors
  public void stopRollers() {
    stopIntake();
    stopMagazine();
  }

  // sets up Intake note state
  public void changeNoteState(NoteState noteState) {
    RobotContainer.noteLifecycle = NoteState.GROUNDINTAKE;
  }

  // sets up Field note state; default state
  public void returnToField(NoteState noteState) {
    RobotContainer.noteLifecycle = NoteState.FIELD;
  }

  // Switches the state that the rollers operate in
  public void rollerSwitch() {
    switch (RobotContainer.noteLifecycle) {

      // Explanation for rising and falling edge code

      // Rising & Falling edge detection code, compares the previous value of the
      // smacna to
      // the current value, this allows for precise commands depending on whether or
      // not there
      // is a rising or falling edge;

      // GROUND INTAKE: Turns low rollers on and switches state when the smacna
      // detects note
      case GROUNDINTAKE:
        setIntake(0.9);
        // intake sensor detects leading edge of note -> Grabbed state
        currentIntakeSmacnaState = intakeSmacnaLeft.get() || intakeSmacnaRight.get();
        if (Helper.detectFallingRisingEdge(previousIntakeSmacnaState, currentIntakeSmacnaState, true)) {
          RobotContainer.noteLifecycle = NoteState.GRABBED;
        }
        previousIntakeSmacnaState = currentIntakeSmacnaState;
        break;

      // GRABBED STATE: Keeps low roller on
      case GRABBED:
        setIntake(0.9);
        setMagazine(0.9);
        // intake sensor detects back edge of the note -> Control state
        currentIntakeSmacnaState = intakeSmacnaLeft.get() || intakeSmacnaRight.get();
        if (Helper.detectFallingRisingEdge(previousIntakeSmacnaState, currentIntakeSmacnaState, false)) {
          RobotContainer.noteLifecycle = NoteState.CONTROL;
        }
        previousIntakeSmacnaState = currentIntakeSmacnaState;
        break;

      // CONTROL STATE: Keeps low roller on
      case CONTROL:
        setIntake(0.9);
        setMagazine(0.9);
        // magazine sensor detects leading edge of note -> Storage state
        currentIntakeSmacnaState = magneticFlap.get();
        if (Helper.detectFallingRisingEdge(previousIntakeSmacnaState, currentIntakeSmacnaState, true)) {
          RobotContainer.noteLifecycle = NoteState.STORAGE;
        }
        previousIntakeSmacnaState = currentIntakeSmacnaState;
        break;

      // STORAGE STATE: Stops low rollers
      case STORAGE:
        stopIntake();
        break;

      // AMPLOADING STATE: Turns on both low and high rollers
      case AMPLOADING:
        // Turns low roller on
        setMagazine(0.1);
        setIntake(0.1);
        // If the magnetic flap moes away from magnet -> Amp state
        currentIntakeSmacnaState = magneticFlap.get();
        previousIntakeSmacnaState = currentIntakeSmacnaState;
        break;

      // SPEAKER STATE: Turns on both high and low rollers and returns to Field state
      // after 5 seconds
      case SPEAKER:
        setIntake(0.1);
        setMagazine(0.1);
      // changes the state to field when button thirteen is true
        if (driverStationButtonPress.button(13).getAsBoolean()) {
            RobotContainer.noteLifecycle = NoteState.FIELD;
        }
        previousIntakeSmacnaState = currentIntakeSmacnaState;
        break;

      // AMP STATE: Reverses low and high rollers when maganetic flap is pushed,
      // returns to field
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

      // EJECT STATE: Turns and keeps both high and low rollers on and turns them off
      // again after 7
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


  @Override
  public void periodic() {
    // Values to showcase the sensor values on the SmartDashboard :)
    SmartDashboard.putBoolean("intakeSmacnaLeft", intakeSmacnaLeft.get());
    SmartDashboard.putBoolean("magazineSmacna", magneticFlap.get());
    SmartDashboard.putBoolean("intakeSmacnaRight", intakeSmacnaRight.get());
    SmartDashboard.putBoolean("shooterSmacna", shooterSmacna.get());

  }

}

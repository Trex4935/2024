package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.extension.FlippedDIO;
import frc.robot.extension.Helper;
import frc.robot.extension.NoteState;
import frc.robot.extension.SparkMax;

public class Rollers extends SubsystemBase {
  // Creating roller motors
  CANSparkMax intake, magazine;

  // Creating Sensors; ID's shown in IDguide.md
  public FlippedDIO magneticFlap, shooterSmacna, dustpanSmacna;
  public DigitalInput storageButton;
  private int i;

  Timer timer;

  public Rollers() {
    // News up motor objects
    intake = SparkMax.createDefaultCANSparkMax(4);
    intake.setInverted(true);
    magazine = SparkMax.createDefaultCANSparkMax(5);

    // Sensor Objects
    dustpanSmacna = new FlippedDIO(6);
    magneticFlap = new FlippedDIO(2);
    shooterSmacna = new FlippedDIO(3);
    storageButton = new FlippedDIO(1);

    // News up a timer object
    timer = new Timer();
  }

  /** Sets the intake motor's speed */
  public void setIntake(double speed) {
    intake.set(speed);
  }

  /** Sets the magazine motor's speed */
  public void setMagazine(double speed) {
    magazine.set(speed);
  }

  /** Sets both roller motors */
  public void setRollers(double intakeSpeed, double magSpeed) {
    intake.set(intakeSpeed);
    magazine.set(magSpeed);
  }

  /** Stops the intake motor */
  public void stopIntake() {
    intake.stopMotor();
  }

  /** Stops the magazine motor */
  public void stopMagazine() {
    magazine.stopMotor();
  }

  /** Stop both roller motors */
  public void stopRollers() {
    stopIntake();
    stopMagazine();
  }

  /** Changes the note's life cycle to its desired state */
  public void changeNoteState(NoteState noteState) {
    RobotContainer.noteLifecycle = noteState;
    // System.out.println(noteState);
  }

  /** Switches the state that the rollers operate in */
  public void rollerSwitch() {
    boolean currentDustpanSmacnaState = false;
    boolean previousDustpanSmacnaState = false;
    // System.out.println(RobotContainer.noteLifecycle);

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
        setMagazine(0.9);
        // intake sensor detects leading edge of note -> Grabbed state
        currentDustpanSmacnaState = dustpanSmacna.get();
        if (currentDustpanSmacnaState && i > 15) {
          RobotContainer.noteLifecycle = NoteState.GRABBED;
        }
        if (storageButton.get()) {
          RobotContainer.noteLifecycle = NoteState.STORAGE;
        }
        previousDustpanSmacnaState = currentDustpanSmacnaState;
        i++;

        break;
        // HUMAN INTAKE STATE: Run magazine backwards
      case SOURCE:
        setMagazine(-0.4);
        if (storageButton.get()) {
          RobotContainer.noteLifecycle = NoteState.FIELD;
        }
        break;

        // GRABBED STATE: Keeps low roller on
      case GRABBED:
        setIntake(0.9);
        // intake sensor detects back edge of the note -> Control state
        currentDustpanSmacnaState = dustpanSmacna.get();
        if (currentDustpanSmacnaState != previousDustpanSmacnaState) {
          RobotContainer.noteLifecycle = NoteState.CONTROL;
        }
        previousDustpanSmacnaState = currentDustpanSmacnaState;
        break;

        // CONTROL STATE: Keeps low roller on
      case CONTROL:
        setIntake(0.9);
        setMagazine(0.9);
        if (storageButton.get()) {
          RobotContainer.noteLifecycle = NoteState.STORAGE;
        }

        break;

        // STORAGE STATE: Stops low rollers
      case STORAGE:
        stopIntake();
        stopMagazine();
        break;

        // SPEAKER STATE: Turns on both high and low rollers and returns to Field state
        // after 5 seconds
      case SPEAKER:
        if (Pivot.pivotAtAngle && Shooter.speedState) {
          setMagazine(0.9);
        }
        break;

        // AMP STATE: Reverses low and high rollers when maganetic flap is pushed,
      case AMP:
        setIntake(0.25);
        setMagazine(0.25);
        currentDustpanSmacnaState = dustpanSmacna.get();
        if (Helper.detectFallingRisingEdge(
            previousDustpanSmacnaState, currentDustpanSmacnaState, false)) {
          timer.start();
          if (timer.hasElapsed(5)) {
            RobotContainer.noteLifecycle = NoteState.FIELD;
            timer.reset();
          }
        }
        previousDustpanSmacnaState = currentDustpanSmacnaState;
        break;

        // EJECT STATE: Turns and keeps both high and low rollers on and turns them off
        // again after 7
        // seconds
      case EJECT:
        magazine.set(0.1);
        intake.set(0.1);
        if (Helper.detectFallingRisingEdge(
            previousDustpanSmacnaState, currentDustpanSmacnaState, false)) {
          timer.start();
          if (timer.hasElapsed(7)) {
            RobotContainer.noteLifecycle = NoteState.FIELD;
            timer.reset();
          }
        }
        break;
      case TRAP:
        if (Pivot.pivotAtAngle) {
          setMagazine(0.9);
        }
        break;
        // Keeps rollers off for magazine and intake
      default:
        stopMagazine();
        stopIntake();
        i = 0;
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addStringProperty(
        "currentNoteLifeCycle", () -> RobotContainer.noteLifecycle.toString(), null);
    builder.addBooleanProperty("magazineSmacna", () -> magneticFlap.get(), null);
    builder.addBooleanProperty("dustpanSmacna", () -> dustpanSmacna.get(), null);
    builder.addBooleanProperty("shooterSmacna", () -> shooterSmacna.get(), null);
    builder.addBooleanProperty("storageButton", () -> storageButton.get(), null);
    builder.addDoubleProperty("Intake Motor Speed", () -> intake.get(), null);
    builder.addDoubleProperty("Magazine Motor Speed", () -> magazine.get(), null);
    builder.addDoubleProperty("Intake Motor Temp", () -> intake.getMotorTemperature(), null);
    builder.addDoubleProperty("Magazine Motor Temp", () -> magazine.getMotorTemperature(), null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.extension.SparkMax;
import frc.robot.RobotContainer;
import frc.robot.extension.FlippedDIO;
import frc.robot.extension.NoteState;
import frc.robot.extension.Helper;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;

public class Rollers extends SubsystemBase {
  // Intake refers to the rollers collecting into the dustpan
  CANSparkMax intake;
  CANSparkMax magazine;

  // public FlippedDIO dustpanSmacnaLeft;
  public FlippedDIO dustpanSmacna;

  // Creating Sensors; ID's shown in IDguide.md
  public FlippedDIO intakeSmacnaLeft;
  public FlippedDIO intakeSmacnaRight;
  public FlippedDIO magneticFlap;
  public FlippedDIO shooterSmacna;
  public DigitalInput storageButton;
  boolean currentIntakeSmacnaState;

  boolean previousIntakeSmacnaState;
  CommandGenericHID driverStationButtonPress;

  Timer timer;

  public Rollers() {
    // News up motor objects
    intake = SparkMax.createDefaultCANSparkMax(4);
    intake.setInverted(true);
    magazine = SparkMax.createDefaultCANSparkMax(5);

    // Sensor Objects
    // dustpanSmacnaLeft = new FlippedDIO(0);
    dustpanSmacna = new FlippedDIO(8);
    magneticFlap = new FlippedDIO(2);
    shooterSmacna = new FlippedDIO(3);
    // Assuming that storageButton replaces magazineSmacna
    storageButton = new FlippedDIO(1);

    // News up a timer object
    timer = new Timer();
  }

  // sets intake speed
  public void setIntake(double speed) {
    intake.set(speed);
  }

  // stops intake motor
  public void stopIntake() {
    intake.stopMotor();
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
  public void setRollers(double intakeSpeed, double magSpeed) {
    intake.set(intakeSpeed);
    magazine.set(magSpeed);
  }

  // stops rollers
  public void stopRollers() {
    stopIntake();
    stopMagazine();
  }

  // sets up Intake note state
  public void changeNoteState(NoteState noteState) {
    RobotContainer.noteLifecycle = noteState;
  }

  // sets up Field note state; default state
  public void returnToField(NoteState noteState) {
    RobotContainer.noteLifecycle = NoteState.FIELD;
  }

  // Switches the state that the rollers operate in
  public void rollerSwitch() {
    boolean currentDustpanSmacnaState = false;
    boolean previousDustpanSmacnaState = false;
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
        currentDustpanSmacnaState = dustpanSmacna.get();
        if (currentDustpanSmacnaState != previousDustpanSmacnaState) {
          RobotContainer.noteLifecycle = NoteState.GRABBED;
        }
        previousDustpanSmacnaState = currentDustpanSmacnaState;
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

      // AMPLOADING STATE: Turns on both low and high rollers
      case AMPLOADING:
        // Turns low roller on
        setMagazine(0.1);
        setIntake(0.1);
        // If the magnetic flap moves away from magnet -> Amp state
        currentDustpanSmacnaState = magneticFlap.get();
        previousDustpanSmacnaState = currentDustpanSmacnaState;
        break;

      // SPEAKER STATE: Turns on both high and low rollers and returns to Field state
      // after 5 seconds
      case SPEAKER:
        setMagazine(0.5);

        // AMP STATE: Reverses low and high rollers when maganetic flap is pushed,
      case AMP:
        setIntake(-0.1);
        setMagazine(-0.1);
        currentDustpanSmacnaState = dustpanSmacna.get();
        if (Helper.detectFallingRisingEdge(previousDustpanSmacnaState, currentDustpanSmacnaState, false)) {
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
        if (Helper.detectFallingRisingEdge(previousDustpanSmacnaState, currentDustpanSmacnaState, false)) {
          timer.start();
          if (timer.hasElapsed(7)) {
            RobotContainer.noteLifecycle = NoteState.FIELD;
            timer.reset();
          }
        }

        // Keeps rollers off for magazine and intake
      default:
        stopMagazine();
        stopIntake();

    }
    System.out.println(RobotContainer.noteLifecycle.toString());
    System.out.println(storageButton.get());
  }

  public void initSendable(SendableBuilder builder) {
    SmartDashboard.putBoolean("magazineSmacna", magneticFlap.get());
    SmartDashboard.putBoolean("dustpanSmacnaRight", dustpanSmacna.get());
    SmartDashboard.putBoolean("shooterSmacna", shooterSmacna.get());
    SmartDashboard.putBoolean("Storage Button", storageButton.get());
    SmartDashboard.putString("currentNoteLifeCycle", RobotContainer.noteLifecycle.toString());
  }

  @Override
  public void periodic() {
  }

}

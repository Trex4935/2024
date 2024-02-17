package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.extension.SparkMax;
import frc.robot.RobotContainer;
import frc.robot.extension.FlippedDIO;
import frc.robot.extension.NoteState;
import frc.robot.extension.Helper;



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

  Timer timer;

  public Rollers() {
    // random id's and creating motor objects
    lowMagazine = SparkMax.createDefaultCANSparkMax(4);
    lowMagazine.setInverted(true);
    highMagazine = SparkMax.createDefaultCANSparkMax(5);
    rollerState = NoteState.FIELD;
    // Sensor Objects
    intakeSmacna = new FlippedDIO(0);
    magazineSmacna = new FlippedDIO(1);
    magneticFlap = new FlippedDIO(2);
    shooterSmacna = new FlippedDIO(3);

    timer = new Timer();

  }

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

  public void stopLowMagazine() {
    lowMagazine.stopMotor();
    highMagazine.stopMotor();
  }

  public void stopHighMagazine() {
    highMagazine.stopMotor();
  }

  @Override
  public void periodic() {
    //

  }

  public void intakeSwitch() {
    switch (rollerState) {

      case GROUNDINTAKE:
        onLowMagazine(0.1);
        // intake sensor detects leading edge of note -> Grabbed state
        currentValue = intakeSmacna.get();
        if (Helper.detectFallingRisingEdge(previousValue, currentValue, true)){
          RobotContainer.noteLifecycle = NoteState.GRABBED;
        }
        previousValue = currentValue;
        break;

      case GRABBED:
        // Turns the low roller on
        onLowMagazine(0.1);
        // intake sensor detects back edge of the note -> Control state
        currentValue = intakeSmacna.get();
        if (Helper.detectFallingRisingEdge(previousValue, currentValue, false)){
          RobotContainer.noteLifecycle = NoteState.CONTROL;
        }
        previousValue = currentValue;
        break;

      case CONTROL:
        onLowMagazine(0.1);
        // magazine sensor detects leading edge of note -> Storage state
        currentValue = magazineSmacna.get();
        if (Helper.detectFallingRisingEdge(previousValue, currentValue, true)){
          RobotContainer.noteLifecycle = NoteState.STORAGE;
        }
        previousValue = currentValue;
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
        currentValue = magazineSmacna.get();
        previousValue = currentValue;
        break;
      case SPEAKER:
        onLowMagazine(0.1);
        onHighMagazine(0.1);
                currentValue = intakeSmacna.get();
        if (Helper.detectFallingRisingEdge(previousValue, currentValue, false))
        {
          timer.start();
          if (timer.hasElapsed(5))
          {
            RobotContainer.noteLifecycle = NoteState.FIELD;
            timer.reset();
          }         
        }
        previousValue = currentValue;
        break;
      case AMP:
        onLowMagazine(-0.1);
        onHighMagazine(-0.1);
        currentValue = intakeSmacna.get();
        if (Helper.detectFallingRisingEdge(previousValue, currentValue, false))
        {
          timer.start();
          if (timer.hasElapsed(5))
          {
            RobotContainer.noteLifecycle = NoteState.FIELD;
            timer.reset();
          }         
        }
        previousValue = currentValue;
        break;
  
      case EJECT:
        highMagazine.set(0.1);
        lowMagazine.set(0.1);
        if (Helper.detectFallingRisingEdge(previousValue, currentValue, false))
        {
          timer.start();
          if (timer.hasElapsed(5))
          {
            RobotContainer.noteLifecycle = NoteState.FIELD;
            timer.reset();
          }         
        }
        
      default:
        // Turns magazines Off
        stopHighMagazine();
        stopLowMagazine();

    }
  }
}

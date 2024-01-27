package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.extension.SparkMax;
import frc.robot.extension.NoteState;


public class Rollers extends SubsystemBase {
    CANSparkMax lowMagazine;
    CANSparkMax highMagazine;

    NoteState rollerState;




public Rollers(){
    // random id's and creating motor objects
lowMagazine = SparkMax.createDefaultCANSparkMax(9);
highMagazine = SparkMax.createDefaultCANSparkMax(10);
rollerState = NoteState.FIELD;
}


public void onLowMagazine(){
    lowMagazine.set(0.1);
}
public void onHighMagazine(){
   highMagazine.set(0.1); 
}

public void stopLowMagazine(){
    lowMagazine.stopMotor();
}
public void stopHighMagazine(){
  highMagazine.stopMotor();
}
@Override
  public void periodic() {
    //
}

public void intakeSwitch(){
    switch (rollerState) {

      case INTAKE:
        onLowMagazine();
        break;
      case GRABBED:
      // Turns the low roller on
        onLowMagazine();
        break;
      case CONTROL:
        onLowMagazine();
        break;
      case STORAGE:
        // Turns low roller off
        stopLowMagazine();
        break;
      case AMPLOADING:
        // Turns low roller on
        onHighMagazine();
        onLowMagazine();
        break;
      case SPEAKER:
        onLowMagazine();
        onHighMagazine();
        break;

      default:
      // Turns magazines Off
        stopHighMagazine();
        stopLowMagazine();
      
        
    }
  }

}

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


public void onLowMagazine(double speed){
    lowMagazine.set(speed);
}
public void onHighMagazine(double speed){
   highMagazine.set(speed); 
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
        onLowMagazine(0.1);
        break;
      case GRABBED:
      // Turns the low roller on
        onLowMagazine(0.1);
        break;
      case CONTROL:
        onLowMagazine(0.1);
        break;
      case STORAGE:
        // Turns low roller off
        stopLowMagazine();
        break;
      case AMPLOADING:
        // Turns low roller on
        onHighMagazine(0.1);
        onLowMagazine(0.1);
        break;
      case SPEAKER:
        onLowMagazine(0.1);
        onHighMagazine(0.1);
        break;

      default:
      // Turns magazines Off
        stopHighMagazine();
        stopLowMagazine();
      
        
    }
  }

}

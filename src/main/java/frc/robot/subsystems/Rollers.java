package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.extension.SparkMax;

public class Rollers extends SubsystemBase {
    CANSparkMax lowMagazine;
    CANSparkMax highMagazine;




public Rollers(){
    // random id's and creating motor objects
lowMagazine = SparkMax.createDefaultCANSparkMax(9);
highMagazine = SparkMax.createDefaultCANSparkMax(10);
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

}

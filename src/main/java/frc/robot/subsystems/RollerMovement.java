package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.extension.SparkMax;

public class RollerMovement extends SubsystemBase {
    CANSparkMax rollermotor1;
    CANSparkMax lowmagazine;
    CANSparkMax highmagazine;
double x;
double y;


public RollerMovement(){
    // random id's and creating motor objects
rollermotor1 = SparkMax.createDefaultCANSparkMax(8);
lowmagazine = SparkMax.createDefaultCANSparkMax(9);
highmagazine = SparkMax.createDefaultCANSparkMax(10);
}

// Roller movement speed
public void onRollerMotor(double speed){
    rollermotor1.set(speed);
    
}
public void onLowMagazine(double speed){
    lowmagazine.set(speed);
}
public void onHighMagazine(double speed){
   highmagazine.set(speed); 
}



    // Stops both roller motors 
public void stopRollerMotor(){
  rollermotor1.stopMotor();
 

}
public void stopLowMagazine(){
    lowmagazine.stopMotor();
}
public void stopHighMagazine(){
  highmagazine.stopMotor();
}
@Override
  public void periodic() {
    //
}

}

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.extension.SparkMax;

public class RollerMovement extends SubsystemBase {
    CANSparkMax rollermotor1;
    CANSparkMax rollermotor2;




public RollerMovement(){
    // random id's and creating motor objects
rollermotor1 = SparkMax.createDefaultCANSparkMax(8);
rollermotor2 = SparkMax.createDefaultCANSparkMax(9);
}

// Roller movement speed
public void RollerSpeed(){
rollermotor1.set(0.1);
rollermotor2.set(0.1);
}

    // Stops both roller motors 
public void stopRollerMotor(){
rollermotor1.stopMotor();
rollermotor2.stopMotor();   

}


@Override
  public void periodic() {
    //
}
}

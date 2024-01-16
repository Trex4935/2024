package frc.robot.extension;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.SparkPIDController.AccelStrategy;

public class SparkMax {
    /**
     * Creates a default CAN Brushless SparkMax motor.
     * 
     * @param CANID CAN ID of the sparkmax.
     * 
     * @return CANSparkMax motor object
     */
    public static CANSparkMax createDefaultCANSparkMax(int CANID) {
        CANSparkMax sparkValue = new CANSparkMax(CANID, CANSparkLowLevel.MotorType.kBrushless);
        sparkValue.setIdleMode(IdleMode.kBrake);
        return sparkValue;
    }

    /**
     * Adds SmartMotion functionality and a PID controller
     * 
     * @param motorObject
     *                        The motor object
     * @param kP
     *                        The proportional gain
     * @param kI
     *                        The integral gain
     * @param kD
     *                        The derivative gain
     * @param kFF
     *                        The feed forward
     * @param minVelocity
     *                        The minimum velocity desired
     * @param maxVelocity
     *                        The maximum velocity desired
     * @param maxAcceleration
     *                        The maximum acceleration desired
     * @param error
     *                        The amount of closed-loop error allowed
     */
    public static CANSparkMax configPIDwithSmartMotion(CANSparkMax motorObject, double kP, double kI, double kD,
            double kFF, double minVelocity, double maxVelocity, double maxAcceleration, double error) {

        /* Creates a PID controller for the SparkMax. */
        SparkPIDController SparkMaxPID = motorObject.getPIDController();

        /* Enables SmartMotion for the PID controller */
        SparkMaxPID.setReference(0, CANSparkMax.ControlType.kSmartMotion, 0);

        /*
         * Sets the acceleration strat (May not do anything since trapezoidal is the
         * only one now?)
         */
        SparkMaxPID.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);

        /* Sets motion constraints for the PID controller */
        SparkMaxPID.setSmartMotionMinOutputVelocity(minVelocity, 0);
        SparkMaxPID.setSmartMotionMaxVelocity(maxVelocity, 0);
        SparkMaxPID.setSmartMotionMaxAccel(maxAcceleration, 0);
        SparkMaxPID.setSmartMotionAllowedClosedLoopError(error, 0);

        /* Gives values to PID controllers */
        SparkMaxPID.setP(kP);
        SparkMaxPID.setI(kI);
        // Not sure if we need to do something with this
        SparkMaxPID.setIZone(0, 0);
        SparkMaxPID.setD(kD);
        SparkMaxPID.setFF(kFF);

        return motorObject;
    }
}
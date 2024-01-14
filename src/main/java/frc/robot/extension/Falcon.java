// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.extension;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

/** Add your docs here. */
public class Falcon {

    private static final double kTimeout = 0.020;

    // Default configuration profile for a falcon
    public static class DefaultConfiguration {

        // private static final String NeutralMode = null;

        // private static final String SensorInitializationStrategy = null;

        // Set Motor Brake mode
        public NeutralModeValue neutralMode = NeutralModeValue.Brake;

        // Input dead band
        public double neutralDeadband = 0.04;

        // Motor ramp when using open loop
        public double openLoopRamp = 0;

        // Set motor limits
        //// normal output forward and reverse = 0% ... i.e. stopped
        public double nominalOutputForward = 0;
        public double nominalOutputReverse = 0;

        //// Max output forward and reverse = 100%
        public double peakOutputForward = 1;
        public double peakOutputReverse = -1;

        // Current limit controls
        public boolean enableCurrentLimit = false;
        public CurrentLimitsConfigs currLimitCfg = new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(enableCurrentLimit)
            .withSupplyCurrentLimit(20)
            .withSupplyCurrentThreshold(60)
            .withSupplyTimeThreshold(0.2);


        // Encoder based limit switches
        public SoftwareLimitSwitchConfigs softwareLimSwitchCfg = new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitEnable(enableCurrentLimit)
        .withForwardSoftLimitThreshold(0)
        .withReverseSoftLimitEnable(enableCurrentLimit)
        .withReverseSoftLimitThreshold(0);
        public boolean enableSoftLimit = false;
        public int forwardSoftLimit = 0;
        public int reverseSoftLimit = 0;



    }

    private static DefaultConfiguration defaultConfig = new DefaultConfiguration();

    //
    /**
     * create a CANTalon with the default (out of the box) configuration
     *
     * @param id
     *           CAN ID of the motor to configure
     * 
     * @return Configured TalonFX motor
     */


    public static TalonFX createDefaultFalcon(int id) {
        return createFalcon(id, defaultConfig, false);
    }

    //
    /**
     * create a CANTalon with the default (out of the box) configuration
     *
     * @param id
     *                    CAN ID of the motor to configure
     * 
     * @param invertMotor
     *                    If motor direction should be inverted
     * 
     * @return Configured TalonFX motor
     */
    public static TalonFX createDefaultFalcon(int id, boolean invertMotor) {
        return createFalcon(id, defaultConfig, invertMotor);
    }

    /**
     * Configures a Falcon with a configuration
     *
     * @param id
     *               CAN ID of the motor to configure
     * 
     * @param config
     *               Config object to use to set values on the motor
     * 
     * @return Configured TalonFX motor
     */
    public static TalonFX createFalcon(int id, DefaultConfiguration config, boolean invertMotor) {
        TalonFX falcon = new TalonFX(id);

        // Set to default and config the basics
        falcon.getConfigurator().apply(new TalonFXConfiguration());

        falcon.setInverted(invertMotor);
        falcon.setControl(new VoltageOut(0));
        falcon.setNeutralMode(config.neutralMode);
        falcon.getConfigurator().apply(new OpenLoopRampsConfigs().withVoltageOpenLoopRampPeriod(config.openLoopRamp));
        

        // set nominal and peak output
       

        // setup supply current limit
        falcon.getConfigurator().apply(config.currLimitCfg);

        // Setup sensor
        falcon.setPosition(0);

        // Configured encoder based limit switch
        falcon.getConfigurator().apply(config.softwareLimSwitchCfg);

        // Return the configured motor object
        return falcon;
    }

    /**
     * Configures a Falcon with a PID configuration in the 0 slot
     *
     * @param motorObject
     *                    TalonFX object to configure
     * @param kP
     *                    Double value of kP portion of PID
     * 
     * @param kI
     *                    Double value of kI portion of PID
     * 
     * @param kD
     *                    Double value of kD portion of PID
     * 
     * @param kF
     *                    Double value of KF portion of PID
     * 
     * 
     * @return Configured TalonFX motor with PID in ID 0
     */
    public static TalonFX configurePID(TalonFX motorObject, double kP, double kI, double kD, double kF) {

        // PID configs
        // setting up the pid
        // Set kP(proportional); kI(Integral); kD(differential); kF(FeedForward)
        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kV = kF;
        slot0Configs.kP = kP;
        slot0Configs.kI = kI;
        slot0Configs.kD = kD;

        // apply gains, 50 ms total timeout
        motorObject.getConfigurator().apply(slot0Configs, kTimeout);
        return motorObject;
    }

    /**
     * Configures a Falcon with MotionMagic configuration
     *
     * @param motorObject
     *                       TalonFX object to configure
     * @param kP
     *                       Double value of kP portion of PID
     * 
     * @param kI
     *                       Double value of kI portion of PID
     * 
     * @param kD
     *                       Double value of kD portion of PID
     * 
     * @param kF
     *                       Double value of KF portion of PID
     * 
     * @param CruiseVelocity
     *                       Double cruise speed of motor during motion
     * 
     * @param Acceleration
     *                       Double acceleration to get to cruise speed
     * 
     * @return Configured TalonFX motor
     */
    /* public static TalonFX configMotionMagic(TalonFX motorObject, double kP, double kI, double kD, double kF,
            double CruiseVelocity, double Acceleration) {

        // Auxilary motor
        motorObject.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0,
                kTimeout);
        motorObject.configNeutralDeadband(0.001, kTimeout);
        motorObject.configOpenloopRamp(0);

        // Set Motion Magic gains in slot0 - see documentation
        motorObject.selectProfileSlot(0, 0);
        motorObject.config_kF(0, kF, kTimeout);
        motorObject.config_kP(0, kP, kTimeout);
        motorObject.config_kI(0, kI, kTimeout);
        motorObject.config_kD(0, kD, kTimeout);

        // Set acceleration and vcruise velocity - see documentation
        motorObject.configMotionCruiseVelocity(CruiseVelocity, kTimeout);
        motorObject.configMotionAcceleration(Acceleration, kTimeout);

        // Zero the sensor once on robot boot up
        motorObject.setSelectedSensorPosition(0, 0, kTimeout);

        // integral Zone
        motorObject.config_IntegralZone(0, 200);

        return motorObject;
    } */
}
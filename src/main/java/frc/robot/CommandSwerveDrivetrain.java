package frc.robot;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.generated.TunerConstants;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
        // Configure supply current limits on the swerve modules
        for (int modIndex = 0; modIndex < 4; modIndex++) {
            SwerveModule module = getModule(modIndex);
            // Drive motor limits
            CurrentLimitsConfigs dconfigs = new CurrentLimitsConfigs();
            module.getDriveMotor().getConfigurator().refresh(dconfigs);
            dconfigs.withSupplyCurrentLimit(20);
            dconfigs.withSupplyCurrentThreshold(2);
            dconfigs.withSupplyTimeThreshold(0);
            dconfigs.withSupplyCurrentLimitEnable(true);
            module.getDriveMotor().getConfigurator().apply(dconfigs);
            // rotation motor limits
            CurrentLimitsConfigs sconfigs = new CurrentLimitsConfigs();
            module.getSteerMotor().getConfigurator().refresh(sconfigs);
            sconfigs.withSupplyCurrentLimit(20);
            sconfigs.withSupplyCurrentThreshold(2);
            sconfigs.withSupplyCurrentLimitEnable(true);
            module.getSteerMotor().getConfigurator().apply(sconfigs);

            // Turn on continuous wrap -- Experimental
            // ClosedLoopGeneralConfigs general = new ClosedLoopGeneralConfigs();
            // general.ContinuousWrap = true;
            // module.getSteerMotor().getConfigurator().apply(general);
        }
    }

    /* Gear Ratio and Wrapping Config */
    // swerveAngleFXConfig.Feedback.SensorToMechanismRatio =
    // Constants.Swerve.angleGearRatio;
    // swerveAngleFXConfig.ClosedLoopGeneral.ContinuousWrap = true;

    private void configurePathPlanner() {
			double driveBaseRadius = 0;
			for (var moduleLocation : m_moduleLocations) {
				driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
			}

			AutoBuilder.configureHolonomic(
					() -> this.getState().Pose, // Supplier of current robot pose
					this::seedFieldRelative, // Consumer for seeding pose against auto
					this::getCurrentRobotChassisSpeeds,
					(speeds) -> this.setControl(autoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the
																																				// robot
					new HolonomicPathFollowerConfig(new PIDConstants(10, 0, 0),
							new PIDConstants(10, 0, 0),
							TunerConstants.kSpeedAt12VoltsMps,
							driveBaseRadius,
							new ReplanningConfig()),
					() -> DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red, // Assume the path needs to be
																																										// flipped for Red vs Blue, this is
																																										// normally the case
					this); // Subsystem for requirements
        
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Command getAutoPath(String pathName) {
        return new PathPlannerAuto(pathName);
    }

		/**
		 * A method of auto-alignment that uses the PathPlanner automatic path generation to align
		 * @param aprilTagPose The desired AprilTag Pose
		 * @param offsetArray The offset values for the desired pose
		 * @return An auto-generated command that goes to a desired pose
		 */
		public Command alignWithPathPlanner(Pose2d aprilTagPose, double[] offsetArray) {
			// Creates an offset pose from the offset array
			 Pose2d pose2dOffset = new Pose2d(offsetArray[0], offsetArray[1], Rotation2d.fromDegrees(offsetArray[2]));
			// Gets target values from the tag poses and the offset
			double targetX = aprilTagPose.getX() + pose2dOffset.getX();
			double targetY = aprilTagPose.getY() + pose2dOffset.getY();
			Rotation2d targetTheta = pose2dOffset.getRotation();
			// Makes a target pose
			Pose2d targetPose = new Pose2d(targetX, targetY, targetTheta);
			// Creates and returns an auto-generated pathfinding command
			Command autoCommand = AutoBuilder.pathfindToPose(targetPose, new PathConstraints(1, 1, 1, 1), 0);
			return autoCommand;
		}

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }
}
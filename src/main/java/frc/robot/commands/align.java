// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Constants;

public class align extends Command {
	/** Creates a new DriveToPoseCommand. */
	private static final double TRANSLATION_TOLERANCE = 0.1;
	private static final double THETA_TOLERANCE = Units.degreesToRadians(2.0);

	private static final TrapezoidProfile.Constraints DEFAULT_XY_CONSTRAINTS = new TrapezoidProfile.Constraints(
			3,
			3);
	private static final TrapezoidProfile.Constraints DEFAULT_OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(
			Math.PI,
			Math.PI);

	private final ProfiledPIDController xController, yController, thetaController;

	private final CommandSwerveDrivetrain drivetrain;
	private final Supplier<Pose2d> poseProvider;
	private final Supplier<Pose2d> goalPoseSupplier;
	private final boolean useAllianceColor;
	private SwerveRequest.ApplyChassisSpeeds initialDrive, driveToPoint, finalDrive;

	public align(CommandSwerveDrivetrain drivetrain, Supplier<Pose2d> goalPoseSupplier, Supplier<Pose2d> poseProvider,
			boolean useAllianceColor) {
		// Use addRequirements() here to declare subsystem dependencies.
		this.drivetrain = drivetrain;
		this.goalPoseSupplier = goalPoseSupplier;
		this.poseProvider = poseProvider;
		this.useAllianceColor = useAllianceColor;

		xController = new ProfiledPIDController(0, 0, 0, DEFAULT_XY_CONSTRAINTS);
		yController = new ProfiledPIDController(0, 0, 0, DEFAULT_XY_CONSTRAINTS);
		thetaController = new ProfiledPIDController(0, 0, 0, DEFAULT_OMEGA_CONSTRAINTS);
		thetaController.enableContinuousInput(-Math.PI, Math.PI);

		xController.setTolerance(0.1);
		yController.setTolerance(TRANSLATION_TOLERANCE);
		thetaController.setTolerance(Units.degreesToRadians(2.0));

		initialDrive = new SwerveRequest.ApplyChassisSpeeds();
		driveToPoint = new SwerveRequest.ApplyChassisSpeeds();
		finalDrive = new SwerveRequest.ApplyChassisSpeeds();

		addRequirements(drivetrain);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		Translation2d emptyTranslation = new Translation2d();
		initialDrive = initialDrive.withSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
				emptyTranslation.getX(),
				emptyTranslation.getY(),
				0.0,
				drivetrain.getRotation3d().toRotation2d()));

		drivetrain.applyRequest(() -> initialDrive);

		resetPIDControllers();
		Pose2d pose = goalPoseSupplier.get();
		if (useAllianceColor && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
			Translation2d transformedTranslation = new Translation2d(pose.getX(), Constants.fieldWidth - pose.getY());
			Rotation2d transformedHeading = pose.getRotation();
			pose = new Pose2d(transformedTranslation, transformedHeading);
		}
		xController.setGoal(pose.getX());
		yController.setGoal(pose.getY());
		thetaController.setGoal(pose.getRotation().getRadians());
	}

	public boolean atGoal() {
		return xController.atGoal() && yController.atGoal() && thetaController.atGoal();
	}

	private void resetPIDControllers() {
		Pose2d robotPose = poseProvider.get();
		xController.reset(robotPose.getX());
		yController.reset(robotPose.getY());
		thetaController.reset(robotPose.getRotation().getRadians());
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		Pose2d robotPose = poseProvider.get();

		double xSpeed = xController.calculate(robotPose.getX());
		if (xController.atGoal()) {
			xSpeed = 0;
		}

		double ySpeed = yController.calculate(robotPose.getY());
		if (yController.atGoal()) {
			ySpeed = 0;
		}

		double thetaSpeed = thetaController.calculate(robotPose.getRotation().getRadians());
		if (thetaController.atGoal()) {
			thetaSpeed = 0;
		}

		driveToPoint = driveToPoint.withSpeeds(
				ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, thetaSpeed, robotPose.getRotation()));

		drivetrain.applyRequest(() -> driveToPoint);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		Translation2d emptyTranslation = new Translation2d();
		finalDrive = finalDrive
				.withSpeeds(
						new ChassisSpeeds(
								emptyTranslation.getX(),
								emptyTranslation.getY(),
								0.0))
				.withDriveRequestType(DriveRequestType.OpenLoopVoltage);
		drivetrain.applyRequest(() -> finalDrive);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return atGoal();
	}
}
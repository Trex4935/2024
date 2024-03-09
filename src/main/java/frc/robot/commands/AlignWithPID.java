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
import frc.robot.extension.Alignment;
import frc.robot.extension.LimelightHelpers;

public class AlignWithPID extends Command {
	/** Creates a new DriveToPoseCommand. */

	private static final TrapezoidProfile.Constraints DEFAULT_XY_CONSTRAINTS = new TrapezoidProfile.Constraints(
			3.5,
			3.5);
	private static final TrapezoidProfile.Constraints DEFAULT_OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(
			Math.PI,
			Math.PI);

	private final ProfiledPIDController xController, yController, thetaController;

	private final CommandSwerveDrivetrain drivetrain;
	private final Supplier<Pose2d> goalPoseSupplier;
	private final boolean useAllianceColor, useVision;
	private SwerveRequest.ApplyChassisSpeeds driveToPoint, finalDrive;

	public AlignWithPID(CommandSwerveDrivetrain drivetrain, Supplier<Pose2d> goalPoseSupplier,
			boolean useAllianceColor, boolean useVision) {
		// Use addRequirements() here to declare subsystem dependencies.
		this.drivetrain = drivetrain;
		this.goalPoseSupplier = goalPoseSupplier;
		this.useAllianceColor = useAllianceColor;
		this.useVision = useVision;

		xController = new ProfiledPIDController(0.2, 0, 0, DEFAULT_XY_CONSTRAINTS);
		yController = new ProfiledPIDController(0.2, 0, 0, DEFAULT_XY_CONSTRAINTS);
		thetaController = new ProfiledPIDController(0.2, 0, 0, DEFAULT_OMEGA_CONSTRAINTS);
		thetaController.enableContinuousInput(-Math.PI, Math.PI);

		xController.setTolerance(0.01);
		yController.setTolerance(0.01);
		thetaController.setTolerance(Units.degreesToRadians(2.0));

		driveToPoint = new SwerveRequest.ApplyChassisSpeeds();
		finalDrive = new SwerveRequest.ApplyChassisSpeeds();

		addRequirements(drivetrain);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {

		resetPIDControllers();
		if (useVision) {
			int tagID = (int) LimelightHelpers.getFiducialID("limelight-battery");
			Supplier<Pose2d> visionGoalPoseSupplier = (() -> getOffsetTarget(tagID));
			Pose2d pose = visionGoalPoseSupplier.get();

			xController.setGoal(pose.getX());
			yController.setGoal(pose.getY());
			thetaController.setGoal(pose.getRotation().getRadians());
		} else {
			Pose2d pose = goalPoseSupplier.get();
			if (useAllianceColor && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
				Translation2d transformedTranslation = new Translation2d(pose.getX(), Alignment.fieldWidth - pose.getY());
				Rotation2d transformedHeading = pose.getRotation();
				pose = new Pose2d(transformedTranslation, transformedHeading);
			}

			xController.setGoal(pose.getX());
			yController.setGoal(pose.getY());
			thetaController.setGoal(pose.getRotation().getRadians());
			
		}
	}

	public boolean atGoal() {
		return xController.atGoal() && yController.atGoal() && thetaController.atGoal();
	}

	private void resetPIDControllers() {
		Pose2d robotPose = drivetrain.getState().Pose;
		xController.reset(robotPose.getX());
		yController.reset(robotPose.getY());
		thetaController.reset(robotPose.getRotation().getRadians());
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (useVision) {
			if (!LimelightHelpers.getTV("limelight-battery")) {
				this.cancel();
			}
		}

		Pose2d robotPose = drivetrain.getState().Pose;

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
		System.out.println(xSpeed);
		System.out.println(ySpeed);
		System.out.println(thetaSpeed);

		drivetrain.setControl(driveToPoint);

		System.out.println("Executing");
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

	private Pose2d getOffsetTarget(int id) {
		Pose2d tagPose = Alignment.aprilTagLayout.getTagPose(id).get().toPose2d();
		// TODO: Find all offsets
		switch (id) {
			case 1:
				return getTargetPose(tagPose, 0.320528, 0.879128, -60.0);

			case 2:
				return getTargetPose(tagPose, 0, 0, 0);

			case 3:
				return getTargetPose(tagPose, 0, 0, 0);

			case 4:
				return getTargetPose(tagPose, -1.763, 0, 0);

			case 5, 6:
				return getTargetPose(tagPose, 0, -0.7042, 90.0);

			case 7:
				return getTargetPose(tagPose, 1.763, 0, -180.0);

			case 8:
				return getTargetPose(tagPose, 0, 0, 0);

			case 9:
				return getTargetPose(tagPose, 0, 0, 0);

			case 10:
				return getTargetPose(tagPose, -0.4, 0.879128, -120.0);

			case 11:
				return getTargetPose(tagPose, 0, 0, 0);

			case 12:
				return getTargetPose(tagPose, 0, 0, 0);

			case 13:
				return getTargetPose(tagPose, 0, 0, 0);

			case 14:
				return getTargetPose(tagPose, 0, 0, 0);

			case 15:
				return getTargetPose(tagPose, 0, 0, 0);

			case 16:
				return getTargetPose(tagPose, 0, 0, 0);

			default:
				return getTargetPose(tagPose, 0, 0, 0);
		}
	}

	private Pose2d getTargetPose(Pose2d aprilTagPose, double xOffset, double yOffset, double rotDeg) {
		// Creates an offset pose from the offset array
		Pose2d pose2dOffset = new Pose2d(xOffset, yOffset, Rotation2d.fromDegrees(rotDeg));
		// Gets target values from the tag poses and the offset
		double targetX = aprilTagPose.getX() + pose2dOffset.getX();
		double targetY = aprilTagPose.getY() + pose2dOffset.getY();
		Rotation2d targetTheta = pose2dOffset.getRotation();
		// Makes a target pose
		Pose2d targetPose = new Pose2d(targetX, targetY, targetTheta);
		return targetPose;
	}
}
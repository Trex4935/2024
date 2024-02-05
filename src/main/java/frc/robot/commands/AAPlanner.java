// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotState;


public class AAPlanner extends Command {
  /** Creates a new AAPlaner. */
	CommandSwerveDrivetrain drivetrain;
	private Pose2d goalPose = null;

  // Controllers for translation and rotation
  private final ProfiledPIDController linearController;
  private final ProfiledPIDController thetaController;

  // Store previous velocities for acceleration limiting
  private Translation2d prevLinearVelocity;
  private double prevAngularVelocity;
  public AAPlanner(CommandSwerveDrivetrain drivetrain) {
		this.drivetrain = drivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
	linearController =
        new ProfiledPIDController(
            0,
            0,
            0,
            new TrapezoidProfile.Constraints(2, 2));
    linearController.setTolerance(0.1);

    thetaController =
        new ProfiledPIDController(
            0,
            0,
            0,
            new TrapezoidProfile.Constraints(
                2 * Math.PI, 2 * Math.PI));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.setTolerance(0.05 * Math.PI);}

  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
		
		public void setGoalPose(Pose2d goalPose) {
			this.goalPose = goalPose;
	
			// Reset controllers
			Pose2d currentPose = drivetrain.getState().Pose;
;
			Translation2d linearFieldVelocity = new Translation2d(
				drivetrain.getCurrentRobotChassisSpeeds().vxMetersPerSecond,
				drivetrain.getCurrentRobotChassisSpeeds().vyMetersPerSecond).
				rotateBy(drivetrain.getState().Pose.getRotation());
			Twist2d fieldVelocity = new Twist2d(linearFieldVelocity.getX(), linearFieldVelocity.getY(), drivetrain.getPigeon2().getRate());
			// Linear controller will control to 0 so distance is the measurement
			Rotation2d rotationToGoal =
					goalPose.getTranslation().minus(currentPose.getTranslation()).getAngle();
			double velocity =
					-new Translation2d(fieldVelocity.dx, fieldVelocity.dy)
							.rotateBy(rotationToGoal.unaryMinus())
							.getX();
			linearController.reset(
					currentPose.getTranslation().getDistance(goalPose.getTranslation()), velocity);
			linearController.setGoal(0.0);
			thetaController.reset(currentPose.getRotation().getRadians(), fieldVelocity.dtheta);
	
			prevLinearVelocity = new Translation2d(fieldVelocity.dx, fieldVelocity.dy);
			prevAngularVelocity = fieldVelocity.dtheta;
		}
	
		public ChassisSpeeds update() {
			Pose2d currentPose = drivetrain.getState().Pose;
			Translation2d linearFieldVelocity = new Translation2d(
				drivetrain.getCurrentRobotChassisSpeeds().vxMetersPerSecond,
				drivetrain.getCurrentRobotChassisSpeeds().vyMetersPerSecond).
				rotateBy(drivetrain.getState().Pose.getRotation());
			Twist2d fieldVelocity = new Twist2d(linearFieldVelocity.getX(), linearFieldVelocity.getY(), drivetrain.getPigeon2().getRate());
	
			// Calculate feedback velocities (based on position error).
			double linearVelocityScalar =
					linearController.calculate(
									currentPose.getTranslation().getDistance(goalPose.getTranslation()))
							+ linearController.getSetpoint().velocity;
			Rotation2d rotationToGoal =
					goalPose.getTranslation().minus(currentPose.getTranslation()).getAngle();
			double xVelocity = -linearVelocityScalar * rotationToGoal.getCos();
			double yVelocity = -linearVelocityScalar * rotationToGoal.getSin();
	
			// If current angular velocity is fast enough in current direction continue in direction
			double angularVelocity =
					thetaController.calculate(
									currentPose.getRotation().getRadians(), goalPose.getRotation().getRadians())
							+ thetaController.getSetpoint().velocity;
	
			// Limit linear acceleration
			// Forward limiting and brake limiting
			Translation2d desiredLinearVelocity = new Translation2d(xVelocity, yVelocity);
			Translation2d deltaVelocity = desiredLinearVelocity.minus(prevLinearVelocity);
			double maxDeltaVelocity = 3 * 0.02;
			if (deltaVelocity.getNorm() > maxDeltaVelocity) {
				desiredLinearVelocity =
						prevLinearVelocity.plus(deltaVelocity.times(maxDeltaVelocity / deltaVelocity.getNorm()));
			}
			prevLinearVelocity = new Translation2d(fieldVelocity.dx, fieldVelocity.dy);
	
			ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds(xVelocity, yVelocity, angularVelocity);
			return ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, currentPose.getRotation());
		}
		
		public boolean atGoal() {
			return linearController.atGoal() && thetaController.atSetpoint();
		}
	
		
		

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

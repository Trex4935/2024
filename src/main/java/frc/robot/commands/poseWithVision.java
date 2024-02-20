// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Constants;
import frc.robot.extension.LimelightHelpers;

public class poseWithVision extends Command {
  /** Creates a new poseWithVision. */
  CommandSwerveDrivetrain drivetrain;
  Timer timer = new Timer();
	alignWithPID align;
  public poseWithVision(CommandSwerveDrivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    timer.start();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // If the Limelight has its target, stop the command
    if (!LimelightHelpers.getTV("LL1")) {
			this.cancel();
			return;
		}

		int tagID = (int) LimelightHelpers.getFiducialID("LL1");

		
		align = new alignWithPID(drivetrain, () -> getOffsetTarget(tagID), false);

		align.execute();
		
  }
	
	private Pose2d getOffsetTarget(int id) {
		Pose2d tagPose = Constants.aprilTagLayout.getTagPose(id).get().toPose2d();
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
				return getTargetPose(tagPose, 0.320528, 0.879128, -120.0);

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

	public Pose2d getTargetPose(Pose2d aprilTagPose, double xOffset, double yOffset, double rotDeg) {
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

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
		align.end(interrupted);
	}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return align.isFinished();
  }
}

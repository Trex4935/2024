// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Constants;
import frc.robot.Constants.ALIGNMENT_POSITION;

public class AutoAlignAlt extends Command {
	/** Creates a new align. */
	private final CommandSwerveDrivetrain drivetrain;
	private final Timer timer;
	private final ALIGNMENT_POSITION position;
	private PathPlannerPath path;

	public AutoAlignAlt(CommandSwerveDrivetrain drivetrain, ALIGNMENT_POSITION position) {
		// Use addRequirements() here to declare subsystem dependencies.

		this.drivetrain = drivetrain;
		this.position = position;
		timer = new Timer();
		timer.start();
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		Pose2d robotPose = drivetrain.getState().Pose;
		Pose2d tagLocation = robotPose.nearest(Constants.speakerAprilTags);

		double targetX = tagLocation.getX() + position.offset.getX();
		double targetY = tagLocation.getY() + position.offset.getY();
		Rotation2d targetTheta = position.offset.getRotation();
		Pose2d targetPose = new Pose2d(targetX, targetY, targetTheta);

		Command autoCommand = AutoBuilder.pathfindToPose(targetPose, new PathConstraints(3, 3, 3, 3), 0);
		autoCommand.schedule();
		timer.reset();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}

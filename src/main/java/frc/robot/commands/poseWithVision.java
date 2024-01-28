// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;

public class poseWithVision extends Command {
  /** Creates a new poseWithVision. */
  CommandSwerveDrivetrain drivetrain;
  Vision vision;
  Timer timer = new Timer();
  public poseWithVision(CommandSwerveDrivetrain drivetrain, Vision vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    timer.start();
    addRequirements(vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // If the Limelight has its target, stop the command
    if (!vision.hasTargets()) {
            this.cancel();
            return;
        }

        // If 50 ms have passed, add vision measurement to odometry
        if (timer.hasElapsed(0.050)) {
            double[] botpose = vision.botPose();
            drivetrain.addVisionMeasurement(new Pose2d(botpose[0], botpose[1], Rotation2d.fromDegrees(botpose[5])),
                    Timer.getFPGATimestamp() - botpose[6] / 1000.0);
            timer.reset();
        }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AlignWithPID;
import frc.robot.extension.NoteState;
import frc.robot.extension.PivotAngle;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DustPan;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Rollers;
import frc.robot.subsystems.LEDControl;

public class RobotContainer {

  // News up our subsystems that we use throughout RobotContainer
  private final DustPan dustpan = new DustPan();
  private final Pivot pivot = new Pivot();
  private final Rollers rollers = new Rollers();
  private final Shooter shooter = new Shooter();
  private final Climber climber = new Climber();
  private final LEDControl ledControl = new LEDControl();

  // Sets the default state in the Note Life Cycle
  public static NoteState noteLifecycle;

  // Swerve settings
  private double MaxSpeed = 6; // 6 meters per second desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  // Setting up bindings for necessary control of the swerve drive platform
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  private final CommandXboxController pivController = new CommandXboxController(2);
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  // Make sure things are field centric for swerve
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  // Setting up the brake and pointing function
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  // Swerve Telemetry
  private final Telemetry logger = new Telemetry(MaxSpeed, drivetrain);
	
	// Alternate align command
	// TODO: Tune offset values
	private final AlignWithPID align = new AlignWithPID(drivetrain, () -> 
	getTargetPose(Constants.speakerAprilTag, Constants.speakerOffset), false, false);


  // Creates the autoChooser to use in the sendables
  private final SendableChooser<Command> autoChooser;

  // Setting up the Button Box
  private final CommandGenericHID operatorTestButton = new CommandGenericHID(1);

  private void configureBindings() {

    // Setup Default commands
    dustpan.setDefaultCommand(dustpan.run(() -> dustpan.intakeSwitch()));
    rollers.setDefaultCommand(rollers.run(() -> rollers.rollerSwitch()));
    shooter.setDefaultCommand(shooter.run(() -> shooter.shooterSwitch()));
    pivot.setDefaultCommand(pivot.run(() -> pivot.pivotSwitch()));

    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically

        // Driving with joysticks
        drivetrain.applyRequest(() -> drive.withVelocityX(joystick.getLeftY() * MaxSpeed) // Drive forward with Joystick
            .withVelocityY(joystick.getLeftX() * MaxSpeed) // Drive left with Joystick
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive right with Joystick
        ));

    // Makes a button that slows the speed down when needed
    joystick.leftTrigger().whileTrue(Commands.runEnd(() -> MaxSpeed = 2, () -> MaxSpeed = 6));

    // A button acts as a brake, and turns all wheels inward
    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    // B button saves the current state of the wheels, and when you let go, it
    // reverts back to them.
    joystick.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

		// reset the field-centric heading on left bumper press
		joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
		joystick.rightBumper().onTrue(pivot.stateSwitcher(PivotAngle.Load));
		// joystick.povUp().onTrue(pivot.stateSwitcher(PivotAngle.Amp));


		joystick.povUp().whileTrue(drivetrain.alignWithPathPlanner(Constants.speakerAprilTag, Constants.speakerOffset).andThen(drivetrain.applyRequest(() -> brake)));
		joystick.povDown().whileTrue(drivetrain.alignWithPathPlanner(
			Constants.sourceAprilTag, Constants.sourceOffset).andThen(drivetrain.applyRequest(() -> brake)));
		joystick.povLeft().whileTrue(drivetrain.alignWithPathPlanner(Constants.ampAprilTag, Constants.ampOffset).andThen(drivetrain.applyRequest(() -> brake)));
		joystick.povRight().whileTrue(drivetrain.alignWithPathPlanner(
			drivetrain.getState().Pose.nearest(Constants.stageAprilTags), Constants.stageOffset).andThen(drivetrain.applyRequest(() -> brake)));

		joystick.start().whileTrue(align);

    // Helps run the simulation
    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);

    // Will move the climber, joystick due to change
    joystick.rightTrigger()
        .whileTrue(climber.runEnd(() -> climber.setClimberMotors(.1), () -> climber.stopClimberMotors()));

    // Test button for the manual setting of the pivot PID
    joystick.y().onTrue(pivot.runOnce(() -> pivot.setPivotPosition("Default")));

    // Buttton 8 runs pivot towards battery
    operatorTestButton.button(8)
        .whileTrue(pivot.runEnd(() -> pivot.setPivotMotor(-0.2), () -> pivot.stopPivotMotor()));

    // Button 9 changes state to ground intake
    operatorTestButton.button(9).onTrue(rollers.runOnce(() -> rollers.changeNoteState(NoteState.GROUNDINTAKE)));

    // Button 10 runs pivot towards force field
    operatorTestButton.button(10).whileTrue(pivot.runEnd(() -> pivot.setPivotMotor(0.2), () -> pivot.stopPivotMotor()));

    // Button 11 changes state to field
    operatorTestButton.button(11).onTrue(rollers.runOnce(() -> rollers.returnToField()));

    // Button 12 changes state to amp
    operatorTestButton.button(12).onTrue(rollers.runOnce(() -> rollers.changeNoteState(NoteState.AMP)));

    // Button 13 changes state to speaker
    operatorTestButton.button(13).onTrue(rollers.runOnce(() -> rollers.changeNoteState(NoteState.SPEAKER)));

    operatorTestButton.button(14).onTrue(rollers.runOnce(() -> rollers.changeNoteState(NoteState.SOURCE)));

    // Button 14 runs mag and shooter
    //operatorTestButton.button(14)
    //    .whileTrue(rollers.runEnd(() -> rollers.setRollers(0.7, 0.7), () -> rollers.stopIntake())
    //        .alongWith(shooter.runEnd(() -> shooter.setShooters(0.9, 0.7), () -> shooter.stopShootingMotors())));

    // Test controller for the pivot motion
    pivController.a().whileTrue(pivot.runEnd(() -> pivot.setPivotPosition("Default"), () -> pivot.stopPivotMotor()));
    pivController.b().whileTrue(pivot.runEnd(() -> pivot.setPivotPosition("Amp"), () -> pivot.stopPivotMotor()));
    pivController.x().whileTrue(pivot.runEnd(() -> pivot.setPivotPosition("Speaker"), () -> pivot.stopPivotMotor()));
    pivController.y().whileTrue(pivot.runEnd(() -> pivot.setPivotPosition("Load"), () -> pivot.stopPivotMotor()));
    pivController.rightBumper().whileTrue(climber.runEnd(() -> climber.setClimberMotorOne(.1), () -> climber.stopClimberMotorOne()));
    pivController.leftBumper().whileTrue(climber.runEnd(() -> climber.setClimberMotorTwo(.1), () -> climber.stopClimberMotorTwo()));
    //pivController.rightBumper().whileTrue(climber.runEnd(() -> climber.setClimberMotors(.1), () -> climber.stopClimberMotors()));


  }

  // Sendables to put autoChooser and Pivot Angle in the SmartDashboard.
  public RobotContainer() {
		Constants.updateAprilTagTranslations();
    configureBindings();
    SmartDashboard.putData(dustpan);
    SmartDashboard.putData(rollers);
    SmartDashboard.putData(pivot);
    // TODO: Fix sendable for note life cycle
    // SmartDashboard.putString("currentNoteLifeCycle", getCycle().toString());

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode", autoChooser);
	}

	public Pose2d getTargetPose(Pose2d aprilTagPose, double[] offsetArray) {
		// Creates an offset pose from the offset array
		 Pose2d pose2dOffset = new Pose2d(offsetArray[0], offsetArray[1], Rotation2d.fromDegrees(offsetArray[2]));
		// Gets target values from the tag poses and the offset
		double targetX = aprilTagPose.getX() + pose2dOffset.getX();
		double targetY = aprilTagPose.getY() + pose2dOffset.getY();
		Rotation2d targetTheta = pose2dOffset.getRotation();
		// Makes a target pose
		Pose2d targetPose = new Pose2d(targetX, targetY, targetTheta);
		return targetPose;
  }

  /** Returns the note state cycle */
  public static NoteState getCycle() {
    return noteLifecycle;
  }

  /** Runs autoChooser :) */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}

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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.PoseEstimation;
import frc.robot.Constants.PoseOffset;
import frc.robot.commands.AlignWithPID;
import frc.robot.extension.NoteState;
import frc.robot.extension.PivotAngle;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Rollers;
import frc.robot.subsystems.LEDControl;

public class RobotContainer {
  // News up our subsystems that we use throughout RobotContainer
  private final Elevator elevator = new Elevator();
  private final Intake intake = new Intake();
  private final LEDControl ledControl = new LEDControl();
  private final Pivot pivot = new Pivot();
  private final Shooter shooter = new Shooter();
  private final Vision vision = new Vision("LL1");
  private final Rollers rollers = new Rollers();

  // Sets the default state in the Note Life Cycle
  public static NoteState noteLifecycle = NoteState.FIELD;

  private double MaxSpeed = 1; // 6 meters per second desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  // Setting up bindings for necessary control of the swerve drive platform

  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  private final CommandXboxController operatorButtonBindings = new CommandXboxController(1);
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  // Make sure things are field centric for swerve
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  // Setting up the brake and pointing function
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  // note cycle return
  public static NoteState getCycle() {
    return noteLifecycle;
  }

	private final Telemetry logger = new Telemetry(MaxSpeed);

	private final AlignWithPID align = new AlignWithPID(drivetrain, () -> Constants.speakerAprilTag, false);

  private final SendableChooser<Command> autoChooser;

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically

        // Driving with joysticks
        drivetrain.applyRequest(() -> drive.withVelocityX(joystick.getLeftY() * MaxSpeed) // Drive forward with Joystick
            .withVelocityY(joystick.getLeftX() * MaxSpeed) // Drive left with Joystick
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive right with Joystick
        ));

    // A button acts as a break, and turns all wheels inward
    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    // B button saves the current state of the wheels, and when you let go, it
    // reverts back to them.
    joystick.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

		// reset the field-centric heading on left bumper press
		joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
		joystick.rightBumper().onTrue(pivot.stateSwitcher(PivotAngle.Load));
		// joystick.povUp().onTrue(pivot.stateSwitcher(PivotAngle.Amp));


		joystick.povUp().whileTrue(drivetrain.alignWithPathPlanner(Constants.speakerAprilTag, PoseOffset.SPEAKER).andThen(drivetrain.applyRequest(() -> brake)));
		// joystick.povDown().whileTrue(drivetrain.alignWithPathPlanner(
		// 	Constants.sourceAprilTag, PoseOffset.SOURCE));
		joystick.povLeft().whileTrue(drivetrain.alignWithPathPlanner(Constants.ampAprilTag, PoseOffset.AMP));
		joystick.povRight().whileTrue(drivetrain.alignWithPathPlanner(
			drivetrain.getState().Pose.nearest(Constants.stageAprilTags), PoseOffset.STAGE));

    joystick.rightTrigger()
        .whileTrue(elevator.runEnd(() -> elevator.elevatorMotorsMovements(), () -> elevator.stopElevatorMotors()));

    // Test button for the manual setting of the pivot PID
    joystick.y().onTrue(pivot.runOnce(() -> pivot.setPID("Default")));

    // Helps run the simulation
    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);

    // maps the button bindings for the operator
    operatorButtonBindings.a().onTrue(pivot.stateSwitcher(PivotAngle.Amp));
    operatorButtonBindings.b().onTrue(pivot.stateSwitcher(PivotAngle.Speaker));
    operatorButtonBindings.x().onTrue(pivot.stateSwitcher(PivotAngle.Feed));
    operatorButtonBindings.y().onTrue(pivot.stateSwitcher(PivotAngle.Load));

  }

  // Sendables to put autoChooser and Pivot Angle in the SmartDashboard.
  public RobotContainer() {
		Constants.updateAprilTagTranslations();
    configureBindings();


    SmartDashboard.putString("angle", pivot.returnPivotAngle());

		SmartDashboard.putNumber("OffsetX", PoseEstimation.testSourcePose.getX() - Constants.sourceAprilTag.getX());
		SmartDashboard.putNumber("OffsetY", PoseEstimation.testSourcePose.getY() - Constants.sourceAprilTag.getY());

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode", autoChooser);

	}

  // Runs autoChooser :)
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}

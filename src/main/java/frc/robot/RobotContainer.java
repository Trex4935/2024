// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Autos;
import frc.robot.extension.Alignment;
import frc.robot.extension.NoteState;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimberLeft;
import frc.robot.subsystems.ClimberRight;
import frc.robot.subsystems.DustPan;
import frc.robot.subsystems.LEDControl;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Rollers;
import frc.robot.subsystems.Shooter;

// import frc.robot.commands.AlignWithPID;

public class RobotContainer {

  // News up our subsystems that we use throughout RobotContainer
  private final DustPan dustpan = new DustPan();
  private final Pivot pivot = new Pivot();
  private final Rollers rollers = new Rollers();
  private final Shooter shooter = new Shooter();
  private final ClimberRight climberRight = new ClimberRight();
  private final ClimberLeft climberLeft = new ClimberLeft();
  private final LEDControl ledControl = new LEDControl();

  // Sets the default state in the Note Life Cycle
  public static NoteState noteLifecycle;

  // Swerve settings
  private double MaxSpeed = 6; // 6 meters per second desired top speed
  private double MaxAngularRate =
      1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  // Setting up bindings for necessary control of the swerve drive platform
  private final CommandXboxController driverJoystick = new CommandXboxController(0); // My joystick
  // private final CommandXboxController altJoystick = new
  // CommandXboxController(2);

  // Setting up the Button Box
  private final CommandGenericHID buttonBox = new CommandGenericHID(1);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  // Make sure things are field centric for swerve
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
  // driving in open loop
  // Setting up the brake and pointing function
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  // Swerve Telemetry
  private final Telemetry logger = new Telemetry(MaxSpeed, drivetrain);

  // Alternate align command
  // TODO: Tune offset values
  // private final AlignWithPID align = new AlignWithPID(drivetrain,
  // () -> getTargetPose(Alignment.speakerAprilTag, Alignment.speakerOffset),
  // false, false);

  // Creates the autoChooser to use in the sendables
  // private final SendableChooser<Command> autoChooser;

  private void configureBindings() {

    // Setup Default commands
    dustpan.setDefaultCommand(dustpan.run(() -> dustpan.intakeSwitch()));
    rollers.setDefaultCommand(rollers.run(() -> rollers.rollerSwitch()));
    shooter.setDefaultCommand(shooter.run(() -> shooter.shooterSwitch()));
    pivot.setDefaultCommand(pivot.run(() -> pivot.pivotSwitch()));
    ledControl.setDefaultCommand(ledControl.run(() -> ledControl.ledSwitch()));

    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically

        // Driving with joysticks
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(-driverJoystick.getLeftY() * MaxSpeed) // Drive forward with
                    // Joystick
                    .withVelocityY(
                        -driverJoystick.getLeftX() * MaxSpeed) // Drive left with Joystick
                    .withRotationalRate(
                        -driverJoystick.getRightX() * MaxAngularRate) // Drive right with Joystick
            ));

    // Control climber motors
    driverJoystick
        .rightTrigger()
        .whileTrue(
            climberRight.runEnd(
                () -> climberRight.setClimberMotorOne(0.8),
                () -> climberRight.stopClimberMotorOne()));
    driverJoystick
        .leftTrigger()
        .whileTrue(
            climberLeft.runEnd(
                () -> climberLeft.setClimberMotorTwo(0.8),
                () -> climberLeft.stopClimberMotorTwo()));
    // Makes a button that slows the speed down when needed
    driverJoystick.leftBumper().whileTrue(Commands.runEnd(() -> MaxSpeed = 2, () -> MaxSpeed = 6));

    // A button acts as a brake, and turns all wheels inward
    driverJoystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    // B button saves the current state of the wheels, and when you let go, it
    // reverts back to them.
    driverJoystick
        .b()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    point.withModuleDirection(
                        new Rotation2d(-driverJoystick.getLeftY(), -driverJoystick.getLeftX()))));

    // reset the field-centric heading on left bumper press
    driverJoystick.rightBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    // All alignment commands require the right trigger to function
    // Up on the D-pad automatically aligns to the speaker
    driverJoystick
        .povUp()
        .and(driverJoystick.start())
        .whileTrue(
            drivetrain
                .alignWithPathPlanner(Alignment.speakerAprilTag, Alignment.speakerOffset)
                .andThen(drivetrain.applyRequest(() -> brake)));

    // Down on the D-pad automatically aligns to the source
    driverJoystick
        .povDown()
        .and(driverJoystick.start())
        .whileTrue(
            drivetrain
                .alignWithPathPlanner(Alignment.sourceAprilTag, Alignment.sourceOffset)
                .andThen(drivetrain.applyRequest(() -> brake)));

    // Left on the D-pad automatically aligns to the amp
    driverJoystick
        .povLeft()
        .and(driverJoystick.start())
        .whileTrue(
            drivetrain
                .alignWithPathPlanner(Alignment.ampAprilTag, Alignment.ampOffset)
                .andThen(drivetrain.applyRequest(() -> brake)));

    // Right on the D-pad automatically aligns to the stage
    driverJoystick
        .povRight()
        .and(driverJoystick.start())
        .whileTrue(
            drivetrain
                .alignWithPathPlanner(
                    drivetrain.getState().Pose.nearest(Alignment.stageAprilTags),
                    Alignment.stageOffset)
                .andThen(drivetrain.applyRequest(() -> brake)));

    // The menu button will align using a PID
    // joystick.start().whileTrue(align);

    // Helps run the simulation
    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);

    // Button 1 manually moves the pivot backwards
    buttonBox.button(1).onTrue(rollers.runOnce(() -> pivot.manualPivotBackward()));

    // Button 3 manually moves the pivot forward
    buttonBox.button(3).onTrue(rollers.runOnce(() -> pivot.manualPivotForward()));

    // Button 5 changes state to trap
    buttonBox.button(5).whileTrue(rollers.runOnce(() -> rollers.changeNoteState(NoteState.TRAP)));

    // Button 6 changes state to speaker (front)
    buttonBox
        .button(6)
        .whileTrue(rollers.runOnce(() -> rollers.changeNoteState(NoteState.SPEAKERFRONT)));

    // Button 7 runs the pivot backwards
    buttonBox
        .button(7)
        .whileTrue(pivot.runEnd(() -> pivot.setPivotMotor(-0.2), () -> pivot.stopPivotMotor()));

    // Button 8 changes state to eject
    buttonBox.button(8).whileTrue(rollers.runOnce(() -> rollers.changeNoteState(NoteState.EJECT)));

    // Button 9 changes state to intake
    buttonBox
        .button(9)
        .onTrue(rollers.runOnce(() -> rollers.changeNoteState(NoteState.GROUNDINTAKE)));

    // Button 10 changes state to source
    buttonBox
        .button(10)
        .whileTrue(rollers.runOnce(() -> rollers.changeNoteState(NoteState.SOURCE)));

    // Button 12 changes state to amp
    buttonBox.button(12).onTrue(rollers.runOnce(() -> rollers.changeNoteState(NoteState.AMP)));

    // Button 13 changes state to speaker
    buttonBox.button(13).onTrue(rollers.runOnce(() -> rollers.changeNoteState(NoteState.SPEAKER)));

    // Button 14 changes state to field
    buttonBox.button(14).onTrue(rollers.runOnce(() -> rollers.changeNoteState(NoteState.FIELD)));
  }

  // Sendables to put autoChooser and Pivot Angle in the SmartDashboard.
  public RobotContainer() {

    NamedCommands.registerCommand("Speaker", Autos.speakerCommand);
    NamedCommands.registerCommand("Field", Autos.fieldCommand);
    NamedCommands.registerCommand("Intake", Autos.intakeCommand);

    Alignment.updateAprilTagTranslations();
    configureBindings();
    SmartDashboard.putData(dustpan);
    SmartDashboard.putData(rollers);
    SmartDashboard.putData(pivot);
    SmartDashboard.putData(shooter);
    SmartDashboard.putData(climberRight);
  }

  public Pose2d getTargetPose(Pose2d aprilTagPose, double[] offsetArray) {
    // Creates an offset pose from the offset array
    Pose2d pose2dOffset =
        new Pose2d(offsetArray[0], offsetArray[1], Rotation2d.fromDegrees(offsetArray[2]));
    // Gets target values from the tag poses and the offset
    double targetX = aprilTagPose.getX() + pose2dOffset.getX();
    double targetY = aprilTagPose.getY() + pose2dOffset.getY();
    Rotation2d targetTheta = pose2dOffset.getRotation();
    // Makes a target pose
    Pose2d targetPose = new Pose2d(targetX, targetY, targetTheta);
    return targetPose;
  }

  // Runs Auto
  public Command getAutonomousCommand() {
    return new PathPlannerAuto("Center2");
  }
}

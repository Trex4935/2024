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
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
  private final Climber elevator = new Climber();
  private final LEDControl ledControl = new LEDControl();

  // Sets the default state in the Note Life Cycle
  public static NoteState noteLifecycle = NoteState.FIELD;

  // Swerve settings
  private double MaxSpeed = 6; // 6 meters per second desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
  private double MinSpeed = 3;

  // Setting up bindings for necessary control of the swerve drive platform
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  // Make sure things are feild centric for swerve
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  // Setting up the brake and pointing function
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  // Swerve Telemetry
  private final Telemetry logger = new Telemetry(MaxSpeed);

  // Creates the autoChooser to use in the sendables
  private final SendableChooser<Command> autoChooser;

  // Setting up the Button Box
  private final CommandGenericHID operatorTestButton = new CommandGenericHID(1);

  private void configureBindings() {

    // Setup Default commands
    dustpan.setDefaultCommand(dustpan.run(() -> dustpan.intakeSwitch()));
    rollers.setDefaultCommand(rollers.run(() -> rollers.rollerSwitch()));
    shooter.setDefaultCommand(shooter.run(() -> shooter.shooterSwitch()));
    pivot.setDefaultCommand(pivot.run(() -> pivot.pivotStateMachine(PivotAngle.Default)));

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
    joystick.povUp().onTrue(pivot.stateSwitcher(PivotAngle.Amp));

    joystick.rightTrigger()
        .whileTrue(elevator.runEnd(() -> elevator.elevatorMotorsMovements(), () -> elevator.stopElevatorMotors()));

    // Test button for the manual setting of the pivot PID
    joystick.y().onTrue(pivot.runOnce(() -> pivot.setPID("Default")));

    // Makes a button that slows the speed down when needed
    joystick.leftTrigger().onTrue(drivetrain.applyRequest(() -> drive.withVelocityX(joystick.getLeftY() * MinSpeed) // Drive forward with Joystick
            .withVelocityY(joystick.getLeftX() * MinSpeed) // Drive left with Joystick
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate))); // Drive right with Joystick)
    // Helps run the simulation
    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);

    // maps the button bindings for the operator
    // operatorButtonBindings.a().onTrue(pivot.stateSwitcher(PivotAngle.Amp));
    // operatorButtonBindings.b().onTrue(pivot.stateSwitcher(PivotAngle.Speaker));
    // operatorButtonBindings.x().onTrue(pivot.stateSwitcher(PivotAngle.Feed));
    // operatorButtonBindings.y().onTrue(pivot.stateSwitcher(PivotAngle.Load))

    // Button 14 runs mag and shooter
    operatorTestButton.button(14)
        .whileTrue(rollers.runEnd(() -> rollers.setRollers(0.7, 0.7), () -> rollers.stopIntake())
            .alongWith(shooter.runEnd(() -> shooter.setShooters(0.9, 0.7), () -> shooter.stopShootingMotors())));
    // operatorButtonBindings.x().whileTrue(rollers.runEnd(() ->
    // rollers.onHighMagazine(0.7),() ->
    // rollers.stopHighMagazine()).alongWith(rollers.runEnd(() ->
    // rollers.onLowMagazine(0.9),() -> rollers.stopLowMagazine())));
    // Button 10 runs pivot
    operatorTestButton.button(10).whileTrue(pivot.runEnd(() -> pivot.runPivotMotor(0.2), () -> pivot.stopPivotMotor()));
    // Buttton 8 runs pivot in reverse
    operatorTestButton.button(8)
        .whileTrue(pivot.runEnd(() -> pivot.runPivotMotor(-0.2), () -> pivot.stopPivotMotor()));
    // Button 13 changes state to ground
    operatorTestButton.button(13).onTrue(rollers.runOnce(() -> rollers.changeNoteState(NoteState.SPEAKER)));
    // Button 12 runs magazine
    operatorTestButton.button(12)
        .whileTrue(pivot.runEnd(() -> rollers.setMagazine(0.7), () -> rollers.stopMagazine()));

    // Button 9 changes state to ground intake
    operatorTestButton.button(9).onTrue(rollers.runOnce(() -> rollers.changeNoteState(NoteState.GROUNDINTAKE)));
    // Button 11 changes state to field
    operatorTestButton.button(11).onTrue(rollers.runOnce(() -> rollers.returnToField(NoteState.FIELD)));

    // operatorTestButton.button(12).onTrue(rollers.runOnce(() ->
    // rollers.changeNoteState(NoteState.);))
  }

  // Sendables to put autoChooser and Pivot Angle in the SmartDashboard.
  public RobotContainer() {
    configureBindings();
    SmartDashboard.putData(dustpan);
    SmartDashboard.putData(rollers);
    SmartDashboard.putData(pivot);
    // SmartDashboard.putString("currentNoteLifeCycle", getCycle().toString());
    // SmartDashboard.putString("angle",
    // pivot.returnPivotAngle(PivotAngle.Default));

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode", autoChooser);

    SmartDashboard.putBoolean("DustPan Solenoid", dustpan.getDustPanState());
    // sendable for

  }

  // note cycle return
  public static NoteState getCycle() {
    return noteLifecycle;
  }

  // Runs autoChooser :)
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}

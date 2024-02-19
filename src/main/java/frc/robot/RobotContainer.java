// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.GadgeteerUartClient.GadgeteerConnection;
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

  // Sets the default state in the Note Life Cycle
  private final Rollers rollers = new Rollers();
  public static NoteState noteLifecycle = NoteState.FIELD;

  private double MaxSpeed = 1; // 6 meters per second desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  // Setting up bindings for necessary control of the swerve drive platform

  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  private final CommandGenericHID operatorTestButton = new CommandGenericHID(1);

  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  // Make sure things are feild centric for swerve
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

  // Creates the autoChooser to use in the sendables
  private final SendableChooser<Command> autoChooser;

  private void configureBindings() {
    intake.setDefaultCommand(intake.run(() -> intake.intakeSwitch()));
    rollers.setDefaultCommand(rollers.run(() -> rollers.rollerSwitch()));

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

    operatorTestButton.button(14)
        .whileTrue(rollers.runEnd(() -> rollers.onLowMagalzine(0.3, 0.9), () -> rollers.stopLowMagazine())
            .alongWith(shooter.runEnd(() -> shooter.shooterMovement(0.9, 0.7), () -> shooter.stopShooterMotors())));
    // operatorButtonBindings.x().whileTrue(rollers.runEnd(() ->
    // rollers.onHighMagazine(0.7),() ->
    // rollers.stopHighMagazine()).alongWith(rollers.runEnd(() ->
    // rollers.onLowMagazine(0.9),() -> rollers.stopLowMagazine())));
    operatorTestButton.button(10).whileTrue(pivot.runEnd(() -> pivot.runPivotMotor(), () -> pivot.stopPivotMotor()));
    operatorTestButton.button(8).whileTrue(pivot.runEnd(() -> pivot.reversePivotMotor(), () -> pivot.stopPivotMotor()));
    operatorTestButton.button(12)
        .whileTrue(shooter.runEnd(() -> shooter.setshootingmotor1(0.9), () -> shooter.stopShootingMotor1()));
    operatorTestButton.button(13)
        .whileTrue(shooter.runEnd(() -> shooter.setshootingmotor2(0.7), () -> shooter.stopShootingMotor2()));
    operatorTestButton.button(9).onTrue(rollers.runOnce(() -> rollers.changeNoteState(NoteState.GROUNDINTAKE)));
    // operatorTestButton.button(11).onTrue();
  }

  // Sendables to put autoChooser and Pivot Angle in the SmartDashboard.
  public RobotContainer() {
    configureBindings();

    SmartDashboard.putString("angle", pivot.returnPivotAngle(PivotAngle.Default));

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode", autoChooser);

    SmartDashboard.putBoolean("Intake Solenoid", intake.getIntakeState());
    // sendable for
  }

  // Runs autoChooser :)
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}

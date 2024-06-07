// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.extension.NoteState;

public final class Autos {

  public static Command speakerCommand =
      Commands.runOnce(
          (() -> {
            RobotContainer.noteLifecycle = NoteState.SPEAKER;
            System.out.println("auto SPEAKER");
          }));
  public static Command fieldCommand =
      Commands.runOnce(
          (() -> {
            RobotContainer.noteLifecycle = NoteState.FIELD;
            System.out.println("auto FIELD");
          }));
  public static Command intakeCommand =
      Commands.runOnce(
          (() -> {
            RobotContainer.noteLifecycle = NoteState.GROUNDINTAKE;
            System.out.println("auto INTAKE");
          }));

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}

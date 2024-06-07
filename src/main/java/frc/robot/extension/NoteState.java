// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.extension;

/** Add your docs here. */
public enum NoteState {
  FIELD, // Note is on the field and we are ignoring it
  GROUNDINTAKE, // Note is on the field but is gonna be picked up
  SOURCE, // Note is intaking via the human player station
  GRABBED, // Note has been intaked, Intake sensor sees note
  CONTROL, // Note is inside robot, intake sensor no longer sees note
  STORAGE, // Note is stationary inside robot, magazine sensor sees note
  SPEAKER, // Note is shot out towards speaker
  AMP, // Note is dropped into the amp
  EJECT,
  TRAP,
  READYCLIMB,
  CLIMB,
  SPEAKERFRONT,
  SPEAKERCUSTOM
}

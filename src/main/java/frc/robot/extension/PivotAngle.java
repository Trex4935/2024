// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.extension;

/** Add your docs here. */
// The degrees are based on a circle where 0/360 degrees is on the right
public enum PivotAngle {
    Default, // Deafult Position of the Shooter angled at 180 degrees approximately
    Speaker, // Position used when aiming at the speaker, approximately 45 or 135 degrees
    Amp, // Position used to score in the amp and or trap, 90 degrees approximately
    Feed, // Position used to take in the piece, may be the the same position as speaker
    Load // Optional, unkown if needed yet
}

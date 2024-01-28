// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final double fieldLength = Units.inchesToMeters(651.25);
    public static final double fieldWidth = Units.inchesToMeters(315.5);
    public static final double tapeWidth = Units.inchesToMeters(2.0);
    public static final double aprilTagWidth = Units.inchesToMeters(6.0);

  public static final double FRAME_PERIMETER = 29;
  public static final double CENTER_OFFSET = Units.inchesToMeters(FRAME_PERIMETER / 2.0 + 15.0);

  public static class VisionConstants {
    public static final double LIMELIGHT_HEIGHT = 11.9; //placeholder
    public static final double LIMELIGHT_X_TRANSLATION = 4.75; //placeholder
    public static final double LIMELIGHT_Y_TRANSLATION = 0.0; //placeholder
    public static final double LIMELIGHT_MOUNTING_ANGLE = 15.0; //placeholder


    public static final double CENTER_LIMIT = 0.3;

    public enum Pipeline {
        DEFAULT(0),
        CHUTE(1);

        public final int value;
        Pipeline(int value) {
            this.value = value;
        }
    }
  }
/* 
  // Dimensions for loading zone and substations, including the tape
  public static final class LoadingZone {
    // Region dimensions
    public static final double width = Units.inchesToMeters(99.0);
    public static final double innerX = fieldLength;
    public static final double midX = fieldLength - Units.inchesToMeters(132.25);
    public static final double outerX = fieldLength - Units.inchesToMeters(264.25);
    public static final double leftY = fieldWidth;
    public static final double midY = leftY - Units.inchesToMeters(50.5);
    public static final double rightY = leftY - width;
    public static final Translation2d[] regionCorners =
            new Translation2d[] {
                    new Translation2d(
                            midX, rightY), // Start at lower left next to border with opponent community
                    new Translation2d(midX, midY),
                    new Translation2d(outerX, midY),
                    new Translation2d(outerX, leftY),
                    new Translation2d(innerX, leftY),
                    new Translation2d(innerX, rightY),
            };

    // Double substation dimensions
    public static final double doubleSubstationLength = Units.inchesToMeters(14.0);
    public static final double doubleSubstationX = innerX - doubleSubstationLength;
    public static final double doubleSubstationShelfZ = Units.inchesToMeters(37.375);

    // Single substation dimensions
    public static final double singleSubstationWidth = Units.inchesToMeters(22.75);
    public static final double singleSubstationLeftX =
            fieldLength - doubleSubstationLength - Units.inchesToMeters(88.77);
    public static final double singleSubstationCenterX =
            singleSubstationLeftX + (singleSubstationWidth / 2.0);
    public static final double singleSubstationRightX =
            singleSubstationLeftX + singleSubstationWidth;
    public static final Translation2d singleSubstationTranslation =
            new Translation2d(singleSubstationCenterX, leftY);

    public static final double singleSubstationHeight = Units.inchesToMeters(18.0);
    public static final double singleSubstationLowZ = Units.inchesToMeters(27.125);
    public static final double singleSubstationCenterZ =
            singleSubstationLowZ + (singleSubstationHeight / 2.0);
    public static final double singleSubstationHighZ =
            singleSubstationLowZ + singleSubstationHeight;
}

  public enum ALIGNMENT_POSITION {
    LEFT_DOUBLE_STATION(-CENTER_OFFSET, 1.1, 0.0, 0.0),
    RIGHT_DOUBLE_STATION(-CENTER_OFFSET, -0.9, 0.0, 0.0),
    SINGLE_STATION(-1.72, CENTER_OFFSET, 90.0, 90),
    LEFT(CENTER_OFFSET, -0.53, 180.0, -180.0),
    MIDDLE(CENTER_OFFSET, 0.0, 180.0, 180.0),
    RIGHT(CENTER_OFFSET, 0.53, 180.0, 180.0);

    
    public final Pose2d offset;
    public final Rotation2d heading;

    ALIGNMENT_POSITION(double x, double y, double heading, double rotation) {
        this.offset = new Pose2d(x, y, Rotation2d.fromDegrees(rotation));
        this.heading = Rotation2d.fromDegrees(heading);
    }
  }

    public static List<Pose2d> aprilTagPoses = new ArrayList<Pose2d>(8);
    public static List<Pose2d> allianceAprilTags = new ArrayList<Pose2d>(4);
    public static List<Pose2d> gridAprilTags = new ArrayList<>(3);
    public static Pose2d humanStationAprilTag;
    public static Pose2d chuteAprilTag = new Pose2d(LoadingZone.singleSubstationTranslation, Rotation2d.fromDegrees(0.0));
    public static List<Pose2d> opposingAllianceAprilTags = new ArrayList<Pose2d>(4);

    public static AprilTagFieldLayout aprilTagLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    public static void updateAprilTagTranslations() {

        aprilTagPoses.clear();
        allianceAprilTags.clear();
        gridAprilTags.clear();
        opposingAllianceAprilTags.clear();
        for (int i = 0; i < aprilTagLayout.getTags().size(); i++) {
            aprilTagPoses.add(i, aprilTagLayout.getTagPose(i + 1).get().toPose2d());
        }

        if (DriverStation.getAlliance() == Optional.of(DriverStation.Alliance.Blue)) {

            chuteAprilTag = new Pose2d(LoadingZone.singleSubstationTranslation, Rotation2d.fromDegrees(0.0));
            gridAprilTags.addAll(aprilTagPoses.subList(5, 8));
            allianceAprilTags.addAll(gridAprilTags);
            humanStationAprilTag = aprilTagPoses.get(3);
            allianceAprilTags.add(humanStationAprilTag);


            opposingAllianceAprilTags.addAll(aprilTagPoses.subList(0, 3));
            opposingAllianceAprilTags.add(aprilTagPoses.get(4));

        } else {
            chuteAprilTag = new Pose2d(new Translation2d(LoadingZone.singleSubstationCenterX, 0), Rotation2d.fromDegrees(0.0));
            gridAprilTags.addAll(aprilTagPoses.subList(0, 3));
            allianceAprilTags.addAll(gridAprilTags);
            humanStationAprilTag = aprilTagPoses.get(4);
            allianceAprilTags.add(humanStationAprilTag);

            opposingAllianceAprilTags.addAll(aprilTagPoses.subList(5, 8));
            opposingAllianceAprilTags.add(aprilTagPoses.get(3));
        }
      }
    */
}

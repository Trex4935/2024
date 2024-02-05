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
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
	public static class OperatorConstants {
		public static final int kDriverControllerPort = 0;
	}

	public static final double fieldLength = Units.inchesToMeters(651.25);
	public static final double fieldWidth = Units.inchesToMeters(323.25);
	public static final double aprilTagWidth = Units.inchesToMeters(6.5);

	public static final double FRAME_PERIMETER = 29;
	public static final double CENTER_OFFSET = Units.inchesToMeters(FRAME_PERIMETER / 2.0 + 15.0);

	public static class VisionConstants {
		public static final double LIMELIGHT_HEIGHT = 11.9; // placeholder
		public static final double LIMELIGHT_X_TRANSLATION = 4.75; // placeholder
		public static final double LIMELIGHT_Y_TRANSLATION = 0.0; // placeholder
		public static final double LIMELIGHT_MOUNTING_ANGLE = 15.0; // placeholder

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

	public static final class PoseEstimation {
		public static ArrayList<Pose2d> scoringPositions = new ArrayList<Pose2d>() {
			{
				add(new Pose2d(1.725, 5.5, new Rotation2d(Math.PI)));// 0,
				add(new Pose2d(1.825, 7.5, new Rotation2d(-Math.PI / 2)));// 1, center of grid
			}
		};

		public static final Pose2d hpStation = new Pose2d(15.61, 7.34, new Rotation2d());

		public static final double kPXController = 3; // 3
		public static final double kPYController = 3; // 3
		public static final double kPThetaController = 3.9; // 3.7

	}

	// Dimensions for loading zone and substations, including the tape
	public class FieldConstants {
  public static double fieldLength = Units.inchesToMeters(651.223);
  public static double fieldWidth = Units.inchesToMeters(323.277);
  public static double wingX = Units.inchesToMeters(229.201);
  public static double podiumX = Units.inchesToMeters(126.75);
  public static double startingLineX = Units.inchesToMeters(74.111);

  public static Translation2d ampCenter =
      new Translation2d(Units.inchesToMeters(72.455), Units.inchesToMeters(322.996));


  /** Each corner of the speaker * */
  public static final class Speaker {

    /** Center of the speaker opening (blue alliance) */
    public static Pose2d centerSpeakerOpening =
        new Pose2d(0.0, fieldWidth - Units.inchesToMeters(104.0), new Rotation2d());
  }

  // corners (blue alliance origin)
  public static Translation3d topRightSpeaker =
      new Translation3d(
          Units.inchesToMeters(18.055),
          Units.inchesToMeters(238.815),
          Units.inchesToMeters(13.091));

  public static Translation3d topLeftSpeaker =
      new Translation3d(
          Units.inchesToMeters(18.055),
          Units.inchesToMeters(197.765),
          Units.inchesToMeters(83.091));

  public static Translation3d bottomRightSpeaker =
      new Translation3d(0.0, Units.inchesToMeters(238.815), Units.inchesToMeters(78.324));
  public static Translation3d bottomLeftSpeaker =
      new Translation3d(0.0, Units.inchesToMeters(197.765), Units.inchesToMeters(78.324));

}
	// TODO: Need to know values for limelight position
	public enum PoseOffset {
		RIGHT_DOUBLE_STATION(-CENTER_OFFSET, -0.9, 0.0, 0.0),
		LEFT(CENTER_OFFSET, -0.53, 180.0, -180.0),
		MIDDLE(CENTER_OFFSET, 0.0, 180.0, 180.0),
		RIGHT(CENTER_OFFSET, 0.53, 180.0, 180.0);

		public final Pose2d offset;
		public final Rotation2d heading;

		PoseOffset(double x, double y, double heading, double rotation) {
			this.offset = new Pose2d(x, y, Rotation2d.fromDegrees(rotation));
			this.heading = Rotation2d.fromDegrees(heading);
		}
	}

	// All AprilTag poses
	public static List<Pose2d> aprilTagPoses = new ArrayList<Pose2d>(16);
	// Poses used by your alliance
	public static List<Pose2d> allianceAprilTags = new ArrayList<Pose2d>(8);
	// Poses used by the opposing alliance
	public static List<Pose2d> opposingAllianceAprilTags = new ArrayList<Pose2d>(8);
	// Poses provided speaker
	public static List<Pose2d> speakerAprilTags = new ArrayList<>(2);
	// Poses provided by trap
	public static List<Pose2d> trapAprilTags = new ArrayList<>(3);
	// Poses provided by human player station (source)
	public static List<Pose2d> sourceAprilTags = new ArrayList<>(2);
	// Pose provided by amp
	public static Pose2d ampAprilTag;

	public static AprilTagFieldLayout aprilTagLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

	public static void updateAprilTagTranslations() {

		// Clear lists
		aprilTagPoses.clear();
		allianceAprilTags.clear();
		speakerAprilTags.clear();
		trapAprilTags.clear();
		sourceAprilTags.clear();
		opposingAllianceAprilTags.clear();

		// Return a pose from each AprilTag
		for (int i = 0; i < aprilTagLayout.getTags().size(); i++) {
			aprilTagPoses.add(i, aprilTagLayout.getTagPose(i + 1).get().toPose2d());
		}

		if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {

			sourceAprilTags.addAll(aprilTagPoses.subList(0, 2));
			allianceAprilTags.addAll(sourceAprilTags);

			ampAprilTag = aprilTagPoses.get(5);
			allianceAprilTags.add(ampAprilTag);

			speakerAprilTags.addAll(aprilTagPoses.subList(6, 8));
			allianceAprilTags.addAll(speakerAprilTags);

			trapAprilTags.addAll(aprilTagPoses.subList(13, 16));
			allianceAprilTags.addAll(trapAprilTags);

			opposingAllianceAprilTags.addAll(aprilTagPoses.subList(2, 5));
			opposingAllianceAprilTags.addAll(aprilTagPoses.subList(8, 13));

		} else {
			speakerAprilTags.addAll(aprilTagPoses.subList(2, 4));
			allianceAprilTags.addAll(speakerAprilTags);

			ampAprilTag = aprilTagPoses.get(4);
			allianceAprilTags.add(ampAprilTag);

			sourceAprilTags.addAll(aprilTagPoses.subList(8, 10));
			allianceAprilTags.addAll(sourceAprilTags);

			trapAprilTags.addAll(aprilTagPoses.subList(10, 13));
			allianceAprilTags.addAll(trapAprilTags);

			opposingAllianceAprilTags.addAll(aprilTagPoses.subList(0, 2));
			opposingAllianceAprilTags.addAll(aprilTagPoses.subList(5, 8));
			opposingAllianceAprilTags.addAll(aprilTagPoses.subList(13, 16));
		}
	}

}

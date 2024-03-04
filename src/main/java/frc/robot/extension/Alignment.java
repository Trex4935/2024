// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.extension;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

/** Add your docs here. */
public class Alignment {

	public static final double fieldLength = Units.inchesToMeters(651.223);
	public static final double fieldWidth = Units.inchesToMeters(323.277);


	// Position offsets for field elements
	public static double[] sourceOffset = new double[3];
	public static double[] speakerOffset = new double[3];
	public static double[] ampOffset = new double[3];
	public static double[] stageOffset = new double[3];

	// All AprilTag poses
	public static List<Pose2d> aprilTagPoses = new ArrayList<Pose2d>(16);
	// Poses used by your alliance (extra tags not included)
	public static List<Pose2d> allianceAprilTags = new ArrayList<Pose2d>(6);
	// Poses used by the opposing alliance
	public static List<Pose2d> opposingAllianceAprilTags = new ArrayList<Pose2d>(8);
	// Pose provided by speaker
	public static Pose2d speakerAprilTag;
	// Poses provided by stage
	public static List<Pose2d> stageAprilTags = new ArrayList<>(3);
	// Pose provided by human player station (source)
	public static Pose2d sourceAprilTag;
	// Pose provided by amp
	public static Pose2d ampAprilTag;

	public static AprilTagFieldLayout aprilTagLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

	public static void updateAprilTagTranslations() {

		// Clear pose lists
		aprilTagPoses.clear();
		allianceAprilTags.clear();
		stageAprilTags.clear();
		opposingAllianceAprilTags.clear();


		// Return a pose from each AprilTag
		for (int i = 0; i < aprilTagLayout.getTags().size(); i++) {
			aprilTagPoses.add(i, aprilTagLayout.getTagPose(i + 1).get().toPose2d());
		}

		// Adds AprilTag poses to lists and sets offsets according to alliance
		if ((DriverStation.getAlliance().isEmpty() || 
				DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)) {

			sourceAprilTag = aprilTagPoses.get(0);
			allianceAprilTags.add(sourceAprilTag);

			ampAprilTag = aprilTagPoses.get(5);
			allianceAprilTags.add(ampAprilTag);

			speakerAprilTag = aprilTagPoses.get(6);
			allianceAprilTags.add(speakerAprilTag);

			stageAprilTags.addAll(aprilTagPoses.subList(13, 16));
			allianceAprilTags.addAll(stageAprilTags);

			opposingAllianceAprilTags.addAll(aprilTagPoses.subList(2, 5));
			opposingAllianceAprilTags.addAll(aprilTagPoses.subList(8, 13));

				sourceOffset[0] = 0.320528;
				sourceOffset[1] = 0.879128;
				sourceOffset[2] = -60.0;
					
				speakerOffset[0] = 1.763;
				speakerOffset[1] = 0;
				speakerOffset[2] = -180.0;
					
				ampOffset[0] = 0;
				ampOffset[1] = -0.7042;
				ampOffset[2] = 90.0;
					
				stageOffset[0] = 0;
				stageOffset[1] = 0.53;
				stageOffset[2] = 180.0; 

		} else {
			speakerAprilTag = aprilTagPoses.get(3);
			ampAprilTag = aprilTagPoses.get(4);
			allianceAprilTags.add(ampAprilTag);
			sourceAprilTag = aprilTagPoses.get(9);
			allianceAprilTags.add(sourceAprilTag);

			stageAprilTags.addAll(aprilTagPoses.subList(10, 13));
			allianceAprilTags.addAll(stageAprilTags);

			opposingAllianceAprilTags.addAll(aprilTagPoses.subList(0, 2));
			opposingAllianceAprilTags.addAll(aprilTagPoses.subList(5, 8));
			opposingAllianceAprilTags.addAll(aprilTagPoses.subList(13, 16));

			sourceOffset[0] = 0.320528;
			sourceOffset[1] = 0.879128;
			sourceOffset[2] = -120.0;
				
			speakerOffset[0] = -1.763;
			speakerOffset[1] = 0;
			speakerOffset[2] = 0;
				
			ampOffset[0] = 0;
			ampOffset[1] = -0.7042;
			ampOffset[2] = 90.0;
				
			stageOffset[0] = 0;
			stageOffset[1] = 0.53;
			stageOffset[2] = 180.0;
		}
	}
}

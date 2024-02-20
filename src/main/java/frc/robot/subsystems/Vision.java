// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  private boolean initialized = false;
  private NetworkTableEntry tTarget = null;
	private NetworkTableEntry tid = null;
  private NetworkTableEntry tx = null;
  private NetworkTableEntry ty = null;
  private NetworkTableEntry ta = null;
  private NetworkTableEntry botpose = null;
  private NetworkTableEntry targetpose = null;
  private NetworkTableEntry tl = null;
  private NetworkTableEntry cl = null;
  private Alliance alliance = Alliance.Blue;
  private String limeLightName = "limelight";
  public Vision(String limeLightName) {
    this.limeLightName = limeLightName;
    this.alliance = DriverStation.getAlliance().get();
    NetworkTable table = NetworkTableInstance.getDefault().getTable(limeLightName);
    
    try {
      tTarget = table.getEntry("tv");
			tid = table.getEntry("tid");
      tx = table.getEntry("tx");
      ty = table.getEntry("ty");
      ta = table.getEntry("ta");
      if (this.alliance == Alliance.Blue) {
        botpose = table.getEntry("botpose_wpiblue");
      } else {
        botpose = table.getEntry("botpose_wpired");
      }
      targetpose = table.getEntry("targetpose_robotspace");
      tl = table.getEntry("tl");
      cl = table.getEntry("cl");
    } catch (Exception e) {
      //SmartDashboard.putBoolean("couldn't get nt entries", true);
    }
    initialized = true;
  }


  public void setAlliance(Alliance alliance) {
    this.alliance = alliance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    NetworkTable table = NetworkTableInstance.getDefault().getTable(limeLightName);
    try {
      tTarget = table.getEntry("tv");
      tx = table.getEntry("tx");
      ty = table.getEntry("ty");
      ta = table.getEntry("ta");
      if (this.alliance == Alliance.Blue) {
        botpose = table.getEntry("botpose_wpiblue");
      }
      else {
        botpose = table.getEntry("botpose_wpired");
      }

      targetpose = table.getEntry("targetpose_robotspace");
      tl = table.getEntry("tl");
      cl = table.getEntry("cl");
    } catch (Exception e) {
      return;
    }
  }

  public NetworkTableEntry getEntry(String str) {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry(str);
  }

  public boolean isInitialized() {
    return this.initialized;
  }

  /** Returns if the Limelight has detected any valid targets as a boolean */
  public boolean hasTargets() {
    boolean hits = false;
    //SmartDashboard.putBoolean("isInitialized", isInitialized());
    if (isInitialized()) {
      hits = (getEntry("tv").getDouble(0.0) == 1.0);
    }
    return hits;
  }

    /** Returns if the Limelight has detected any valid targets as a 1 or 0 */
  public double tv() {
    double tv = 0.0;
    if (isInitialized()) {
      tv = tTarget.getDouble(0.0);
    }
    return tv;
  }

  /** Returns the current pose of the robot */
  public double[] botPose() {
    double[] botPose = null;
    if (isInitialized()) {
      botPose = botpose.getDoubleArray(new double[7]);
    }
    return botPose;
  }

  /** Returns the targeting latency */
  public double tl() {
    double tL = 0.0;
    if (isInitialized()) {
      tL = tl.getDouble(0.0);
    }
    return tL;
  }

  //* Returns the capture latency */
  public double cl() {
    double cL = 0.0;
    if (isInitialized()) {
      cL =cl.getDouble(0.0);
    }
    return cL;
  }

  /** Returns if the Limelight has detected any targets */
  public double targetDist() {
    double[] targetPose = null;
    if (isInitialized()) {
      targetPose = targetpose.getDoubleArray(new double[3]);
    }
    Translation3d dist = new Translation3d(targetPose[0], targetPose[1], targetPose[2]);
    return dist.getDistance(new Translation3d());
  }

  /** Returns the horizontal offset from a target */
  public double x() {
    double dx = 0.0;
    if (isInitialized()) {
      dx = tx.getDouble(0.0);
    }
    return dx;
  }

  /** Returns the vertical offset from a target */
  public double y() {
    double dy = 0.0;
    if (isInitialized()) {
      dy = ty.getDouble(0.0);
    }
    return dy;
  }

  /** Returns the target area */
  public double targetArea() {
    double dArea = 0.0;
    if (isInitialized()) {
      dArea = ta.getDouble(0.0);
    }
    return dArea;
  }
}
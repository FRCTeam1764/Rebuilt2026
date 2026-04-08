// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.LimelightHelpers;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LocalizationSubsystem extends SubsystemBase {
  /** Creates a new LocalizationSubsystem. */
  private Field2d field;
  //x, y, 0, 0, 0, yaw
  private double[] arrayPose2d;
  private LimelightSubsystem localLimelight1;
  private LimelightSubsystem localLimelight2;
  private LimelightSubsystem[] limelights;
  private CommandSwerveDrivetrain drivetrain;
  private Pose2d localPose = new Pose2d();

  // every meter
  private double[] shooterWristAngles = {0, 0, 0, 0, 0, 0};
  
  private boolean fieldUse = false;

  public LocalizationSubsystem(CommandSwerveDrivetrain drivetrain, Field2d field, LimelightSubsystem localLimelight1, LimelightSubsystem localLimelight2) {
    this.drivetrain = drivetrain;
    this.field = field;
    this.localLimelight1 = localLimelight1;
    this.localLimelight2 = localLimelight2;
    limelights = new LimelightSubsystem[]{localLimelight1, localLimelight2};
    this.fieldUse = true;
  }

  private double x;
  private double y;
  private double rotation;
  private double hubX = 4.63; //182.1 inches
  private double hubY = 4.03; //158.85

  public Pose2d getPose() {
    return localPose;
  }

  public void updateEstimator() {
    boolean limelightEstimator = false;
    for (LimelightSubsystem limelightSelect : limelights) {
      if (limelightSelect.hasTarget()) {
        SmartDashboard.putString("Estimator", limelightSelect.getLimelightName());
        localPose = LimelightHelpers.getBotPose2d_wpiBlue(limelightSelect.getLimelightName());
        limelightEstimator = true;
        return;
      }
    }
    if (!limelightEstimator) {
      if (!SmartDashboard.getString("Estimator", "N/A").equals("Drivetrain")) {
        drivetrain.resetPose(localPose);
      }
      SmartDashboard.putString("Estimator", "Drivetrain");
      localPose = drivetrain.getState().Pose;
    }
  }

  public void postPoseEstimation() {
    arrayPose2d = LimelightHelpers.pose2dToArray(localPose);
    
    x = arrayPose2d[0];
    y = arrayPose2d[1];

    arrayPose2d[5] = drivetrain.getRobotAngle();

    rotation = arrayPose2d[5];

    localPose = new Pose2d(x, y, Rotation2d.fromDegrees(arrayPose2d[5]));


    field.setRobotPose(localPose);

    SmartDashboard.putNumberArray("RobotPose2d", arrayPose2d);

    SmartDashboard.putNumber("RobotX", x);
    SmartDashboard.putNumber("RobotY", y);
    SmartDashboard.putNumber("Rotation", rotation);
    // √((x-hubX)^2 + (y-hubY)^2) = distance
    SmartDashboard.putNumber("DistanceFromHub", calcHubDistance());
  }

  // public double calcHubAngle() { // 0 to 1
  //   return (Math.atan(Math.abs(y-hubY)/Math.abs(x-hubX))+(Math.PI/2))/(2*Math.PI);
  // }

  public double calcHubDistance() {
    return Math.sqrt(Math.pow(Math.abs(x-hubX), 2)+Math.pow(Math.abs(y-hubY), 2));
  }

  public double calcHubAngle() {
    Translation2d offset = new Translation2d(hubX-x, hubY-y);
    return offset.getAngle().getDegrees();
  }

  public double getShooterWristAngle() {
    int index = (int) (calcHubDistance()/1);
    if (index >= shooterWristAngles.length) {
      index = shooterWristAngles.length-1;
    }
    return shooterWristAngles[index];
  }

  public double getRobotRelHubAngle() {
    return calcHubAngle() - rotation - 45; //TODO: find out what the actual offset is
  }

  @Override
  public void periodic() {
    updateEstimator();
    postPoseEstimation();
    SmartDashboard.putData("LocalizationField", field);
    // This method will be called once per scheduler run
  }
}
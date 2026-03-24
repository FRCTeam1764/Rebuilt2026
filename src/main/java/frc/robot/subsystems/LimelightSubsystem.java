package frc.robot.subsystems;


import java.lang.reflect.Field;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.common.LimelightHelpers;

public class LimelightSubsystem extends SubsystemBase {
  /** Creates a new LimeLight. */

  private String Limelight;
  private double mountAngle; //degrees
  private double elevation; //cm

  public LimelightSubsystem(String LimelightName) {
    /**
     * tx - Horizontal Offset
     * ty - Vertical Offset 
     * ta - Area of target 
     * tv - Target Visible
     */

    Limelight = LimelightName;
    if (Limelight == "limelight-fourtwo") {
      mountAngle = 0;
      elevation = 5;
    } else {
      mountAngle = 15;
      elevation = 5;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber(Limelight + "-A", getTa());
    SmartDashboard.putNumber(Limelight + "-S", getTs());
    SmartDashboard.putNumber("Hot Dogs away from Target", getDistanceToTarget());
  }

  public double getHorizontalAngleOfErrorDegrees(){
    return getTx();
  }

  public double getVerticalAngleOfErrorDegrees(){
    return getTy();
  }

  public String getLimelightName() {
    return Limelight;
  }

  public double getTx() {
    return LimelightHelpers.getTX(Limelight);
  }

  public double getTy() {
    return LimelightHelpers.getTY(Limelight);
  }

  public double getTa() {
    return LimelightHelpers.getTA(Limelight);
  }

  public double getTs() {
    return LimelightHelpers.getTS(Limelight);
  }

  public double getXDistance(){
    return LimelightHelpers.getCameraPose3d_RobotSpace(Limelight).getX();

  }
  public double getZDistance(){
    return LimelightHelpers.getCameraPose3d_RobotSpace(Limelight).getZ();
    
  }
  public void setPipeline(int pipe){
    LimelightHelpers.setPipelineIndex(Limelight, pipe);
  }

  public int getID(){
    return (int) LimelightHelpers.getFiducialID(Limelight);
  }

  public double getDistanceToTarget(){
    double x = LimelightHelpers.getCameraPose3d_TargetSpace(Limelight).getX();
    double y = LimelightHelpers.getCameraPose3d_TargetSpace(Limelight).getZ();
    return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
  }

  public boolean hasTarget(){
    return LimelightHelpers.getTV(Limelight);
  }

  public double getTxAngleRadians(){
    return Units.degreesToRadians(getTx());
  }

  public double getEstDistanceFromHub() {
    // Distance from limelight to goal
    // (goalFromFloor - limelightFromFloor) / Math.tan(mountAngle + angleUp)
    
    return (60 - elevation) / Math.tan(mountAngle + getTy());
  }

  public double getPassingRotation(double distanceToTarget) {
    //distance of april to ramp is (for now) 2m (TODO: FIX)
    // c = a/cos(tx) or just a
    // angle = arcsin(distanceToTarget/c (opposite/hypotenuse))
    return getTx() + 2 * Math.asin(distanceToTarget/getTa());
  }

  //TODO: localization stuff
}
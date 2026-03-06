// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class TurretManager extends SubsystemBase {
  /** Creates a new TurretManager. */

  String turretMode = "IDLE";
  String wristMode = "IDLE";

  boolean hubAimming = false;
  boolean manualAimming = false;

  TurretRev turret;
  ShooterWristRev wrist;
  LocalizationSubsystem local;
  CommandXboxController xbox;
  

  public TurretManager(TurretRev turret, ShooterWristRev wrist, LocalizationSubsystem localization, CommandXboxController xbox) {
    this.turret = turret;
    this.wrist = wrist;
    this.local = localization;
    this.xbox = xbox;
  }

  public void hubAim() {
    turretMode = "HUB_AIM";
    wristMode = "HUB_AIM";
  }

  public void hubAimToggle() {
    if (!hubAimming) {
      hubAim();
      hubAimming = true;
    } else {
      idleMode();
      hubAimming = false;
    }
  }

  public void manualAimToggle() {
    if (!manualAimming) {
      manualAim();
      manualAimming = true;
    } else {
      idleMode();
      manualAimming = false;
    }
  }

  public void turretHubAim(){
    turretMode = "HUB_AIM";
  }

  public void wristHubAim(){
    wristMode = "HUB_AIM";
  }

  public void turretIdle(){
    turretMode = "IDLE";
  }

  public void wristIdle(){
    wristMode = "IDLE";
  }

  public void idleMode () {
    turretMode = "IDLE";
    wristMode = "IDLE";
  }

  public void manualAim () {
    turretMode = "MANUAL_AIM";
    wristMode = "MANUAL_AIM";
  }

  public void setTurret() {
    switch (turretMode) {
      case "HUB_AIM":
        turret.onAnglePosition(local.calcHubAngle());
        break;
      case "MANUAL_AIM":
        turret.onSpeed(xbox.getRightX());
        break;
      case "IDLE":
        turret.stop();
        break;
    }
  }

  public void setWrist() {
    switch (wristMode) {
      case "HUB_AIM":
        wrist.onAnglePosition(30);
        break;
      case "MANUAL_AIM":
        wrist.onSpeed(xbox.getLeftY());
        break;
      case "IDLE":
        wrist.stop();
        break;
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Hub Aimming", hubAimming);
    SmartDashboard.putBoolean("Manual Aiming", manualAimming);
    // This method will be called once per scheduler run
  }
}

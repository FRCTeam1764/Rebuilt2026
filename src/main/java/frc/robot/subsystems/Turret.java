// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CommandConstants;

public class Turret extends SubsystemBase {
  /** Creates a new Turret. */
  TalonFX motor = new TalonFX(CommandConstants.TURRET_ID);
  boolean reset = false;
  public Turret() {}

  public void on(double speed) { 
    if(getPos()<355 && !reset) {
      motor.set(speed);
    } else if (getPos()>=355 || reset) {
      motor.set(CommandConstants.TURRET_BACK_SPEED);
    } else if (reset && getPos()>5) {
      reset = false;
    }
  }

  public double getPos() {
    return motor.getPosition().getValueAsDouble();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

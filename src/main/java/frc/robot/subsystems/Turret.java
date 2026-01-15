// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CommandConstants;
import frc.robot.constants.Constants;

public class Turret extends SubsystemBase {
  /** Creates a new Turret. */
  TalonFX motor = new TalonFX(Constants.TURRET_MOTOR.id, Constants.TURRET_MOTOR.busName);
  boolean reset = false;
  public Turret() {
    
    TalonFXConfiguration turretConfig = new TalonFXConfiguration();
    turretConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    turretConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    turretConfig.MotorOutput.PeakForwardDutyCycle = 0.25;
    turretConfig.MotorOutput.PeakReverseDutyCycle = -0.25;
    turretConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    turretConfig.CurrentLimits.StatorCurrentLimit = 40;

    motor.getConfigurator().apply(turretConfig);
  }

  public void on(double speed) { 
    if(getPos()<355 && !reset) {
      motor.set(CommandConstants.TURRET_SPEED);
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

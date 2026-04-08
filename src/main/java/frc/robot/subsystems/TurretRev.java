// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkSoftLimit;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CommandConstants;
import frc.robot.constants.Constants;


public class TurretRev extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  private final SparkMax turretMotor = new SparkMax(23, SparkLowLevel.MotorType.kBrushless);
  DigitalOutput limitSwitch = new DigitalOutput(2);
  CANcoder turretEncoder = new CANcoder(4);
  boolean left = false;
  boolean switchSides = false;
  boolean pressed = false;
  double cheaterEncoder;

  double turretLeftMax = -1.00; //-1.55
  double turretRightMax = 0.00; //0.75

  PIDController pid1 = new PIDController(1.7, 0, 0);
  PIDController pid2 = new PIDController(2.5, 0, 0);

  double calculation;
  
  public TurretRev() {
  }

  public void onPosition(double desiredPos) { 
    turretMotor.getClosedLoopController().setSetpoint(desiredPos, ControlType.kPosition);
  }

  public void onPositionEx(double desiredPos) {
    if ((getPos() > 300 && calculation>0) || (getPos() < 10 && calculation<0)) {
      turretMotor.set(0);
    } else {
      calculation = pid1.calculate(getPos(), desiredPos);
      SmartDashboard.putNumber("INTAKE_WRIST_PID", calculation);
      turretMotor.set(calculation);
    }
  }

  public void onAngleEx(double desiredPos) {
    if ((getAngle() > 300 && calculation>0) || (getAngle() < 10 && calculation<0)) {
      turretMotor.set(0);
    } else {
      calculation = pid1.calculate(getAngle(), desiredPos);
      SmartDashboard.putNumber("INTAKE_WRIST_PID", calculation);
      turretMotor.set(calculation);
    }
  }

  public void onAnglePosition(double desiredAngle) { 
    turretMotor.getClosedLoopController().setSetpoint(desiredAngle/360, ControlType.kPosition);
  }

  public double getPos() {
    return turretEncoder.getAbsolutePosition().getValueAsDouble();
  }

  public double getAngle() {
    return turretEncoder.getAbsolutePosition().getValueAsDouble()*360;
  }

  public void onSpeed(double speed) {
    if ((speed > 0 && getPos() > turretLeftMax) || (speed < 0 && getPos() < turretRightMax)) {
      turretMotor.set(speed*0.25);
    } else {
      turretMotor.set(0);
    }
  }

  public void on(boolean neg) {
    turretMotor.set(neg ? -CommandConstants.TURRET_SPEED : CommandConstants.TURRET_SPEED);
  }

  public void stop() {
    turretMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("TurretPosition", turretEncoder.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber("TurretTemperature", turretMotor.getMotorTemperature());
    SmartDashboard.putNumber("TurretCurrent", turretMotor.getOutputCurrent());
    SmartDashboard.putNumber("TurretSetpoint", turretMotor.getClosedLoopController().getSetpoint());
    SmartDashboard.putNumber("TurretSpeed", turretEncoder.getVelocity().getValueAsDouble());

    SmartDashboard.putBoolean("Turret Limit Switch", limitSwitch.get());
    SmartDashboard.putBoolean("turret pressed", pressed);

    if (limitSwitch.get() && !pressed) {
      turretEncoder.setPosition(0);
      pressed = true;
    } else if (!limitSwitch.get() && pressed) {
      pressed = false;
    }
  }
}
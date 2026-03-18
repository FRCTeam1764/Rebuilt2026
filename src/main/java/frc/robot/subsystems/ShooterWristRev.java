// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.hal.PowerDistributionVersion;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkSoftLimit;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import static edu.wpi.first.units.Units.Volts;

public class ShooterWristRev extends SubsystemBase {
  /** Creates a new IntakeWrist. */

  SparkMax wristMotor = new SparkMax(5, SparkLowLevel.MotorType.kBrushless);

  public ShooterWristRev() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.inverted(false);
    config.idleMode(SparkBaseConfig.IdleMode.kBrake);
    config.smartCurrentLimit(40);

    ClosedLoopConfig pidConfig = new ClosedLoopConfig();
    pidConfig.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    pidConfig.outputRange(-0.4, 0.4);
    pidConfig.pid(2.8, 0 ,0);
    pidConfig.allowedClosedLoopError(0.001, ClosedLoopSlot.kSlot0);

    // SoftLimitConfig softLimitConfig = new SoftLimitConfig();
    // softLimitConfig.reverseSoftLimit(0.3);
    // softLimitConfig.reverseSoftLimitEnabled(true);
    // softLimitConfig.forwardSoftLimit(0.56);
    // softLimitConfig.forwardSoftLimitEnabled(true);

    config.apply(pidConfig);
    //config.apply(softLimitConfig);

    wristMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  public double getSpeed() {
    return wristMotor.getAbsoluteEncoder().getVelocity();
  }

  public void stop() {
    wristMotor.set(0);
  }

  public void setPos(double desiredPos) {
    wristMotor.getClosedLoopController().setSetpoint(desiredPos, ControlType.kPosition);
    SmartDashboard.putNumber("pid controller", wristMotor.getClosedLoopController().getSetpoint());
  }

  public void onSpeed(double speed) {
    wristMotor.set(speed);
  }

  public double getPos() {
    return wristMotor.getAbsoluteEncoder().getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ShooterWristPosition", wristMotor.getAbsoluteEncoder().getPosition());
    SmartDashboard.putNumber("ShooterWristTemperature", wristMotor.getMotorTemperature());
    SmartDashboard.putNumber("ShooterWristCurrent", wristMotor.getOutputCurrent());
    SmartDashboard.putNumber("ShooterWristSetpoint", wristMotor.getClosedLoopController().getSetpoint());
    SmartDashboard.putNumber("ShooterWristVoltage", wristMotor.getAppliedOutput());
  }
}
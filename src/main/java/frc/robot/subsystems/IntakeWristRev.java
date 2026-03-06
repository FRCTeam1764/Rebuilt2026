// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.VoltageOut;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class IntakeWristRev extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  private final SparkMax wristMotor = new SparkMax(62, SparkLowLevel.MotorType.kBrushless);
  private double wristSpeed = 0.2;
  
  public IntakeWristRev() {

    SparkMaxConfig config = new SparkMaxConfig();
    config.inverted(true);
    config.idleMode(SparkBaseConfig.IdleMode.kBrake);
    //config.smartCurrentLimit(20);

    ClosedLoopConfig pidConfig = new ClosedLoopConfig();
    pidConfig.pid(2, 0, 0, ClosedLoopSlot.kSlot0);
    pidConfig.pid(8, 0, 0, ClosedLoopSlot.kSlot1);

    pidConfig.outputRange(-0.3, 0.1);
    pidConfig.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    config.apply(pidConfig);
    

    // SoftLimitConfig limitConfig = new SoftLimitConfig();
    // limitConfig.forwardSoftLimitEnabled(true);
    // limitConfig.forwardSoftLimit(0.075); //9.83
    // limitConfig.reverseSoftLimitEnabled(true);
    // limitConfig.reverseSoftLimit(0.25);
    // config.apply(limitConfig);

    //config, resets configs to default, configs persist even after motor is power cycled
    wristMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void onPosition(double desiredPos) { 
    if (desiredPos < wristMotor.getAbsoluteEncoder().getPosition()) {
      wristMotor.getClosedLoopController().setSetpoint(desiredPos, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    } else {
      wristMotor.getClosedLoopController().setSetpoint(desiredPos, ControlType.kPosition, ClosedLoopSlot.kSlot1);
    }
  }

  public void onAnglePosition(double desiredAngle) { 
    wristMotor.getClosedLoopController().setSetpoint(desiredAngle/360, ControlType.kPosition);
  }

  public double getPos() {
    return wristMotor.getEncoder().getPosition();
  }

  public void onSpeed(double speed) {
    if (wristMotor.getAbsoluteEncoder().getPosition() <= 0.075 && speed < 0) {
      speed = 0;
    }
    if (wristMotor.getAbsoluteEncoder().getPosition() >= 0.25 && speed > 0) {
      speed = 0;
    }
    wristMotor.set(speed);
  }

  public void on(boolean neg) {
    wristMotor.set(neg ? -wristSpeed : wristSpeed);
  }

  public void stop() {
    wristMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("IntakeWristPosition", wristMotor.getAbsoluteEncoder().getPosition());
    SmartDashboard.putNumber("IntakeWristTemperature", wristMotor.getMotorTemperature());
    SmartDashboard.putNumber("IntakeWristCurrent", wristMotor.getOutputCurrent());
    SmartDashboard.putNumber("IntakeWristSetpoint", wristMotor.getClosedLoopController().getSetpoint());
    SmartDashboard.putNumber("IntakeWristVoltage", wristMotor.getAppliedOutput());
  }
}
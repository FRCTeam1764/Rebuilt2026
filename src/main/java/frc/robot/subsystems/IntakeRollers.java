// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

// Elliott's task
// please inform me if anything is wrong with this
// constants is also missing intake_motor + break_beam, so when we get that added in remove the comments mentioning it or anything related to it
// and of course if i need to change anything about this tell me

public class IntakeRollers extends SubsystemBase {
  /** Creates a new IntakeRollers. */
  private TalonFX m_intakeMotor;
  DoubleLogEntry currentLog;
  BooleanLogEntry limitSwitchLog;

  public IntakeRollers() {
    m_intakeMotor = new TalonFX(Constants.INTAKE_MOTOR.id, Constants.INTAKE_MOTOR.busName);


    TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
    intakeConfig.MotorOutput.Inverted =  InvertedValue.Clockwise_Positive;
    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    intakeConfig.MotorOutput.PeakForwardDutyCycle = 0.9;
    intakeConfig.MotorOutput.PeakReverseDutyCycle = -0.9;
    intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    intakeConfig.CurrentLimits.StatorCurrentLimit = 60;

    
    m_intakeMotor.getConfigurator().apply(intakeConfig);

  }

  double negative;
  public void wheelsIntake(double speed) {
    
    // redundant
    /*if (speed < 0) {
      negative = -1;
    } else {
      negative = 1;
    }*/

     if (speed < 0) {
       m_intakeMotor.set(Math.min(speed, 0));
     } else {
       m_intakeMotor.set(getPercentFromBattery(speed));
     }

  }
  public double getPercentFromBattery(double speed){
    return speed * 12 / RobotController.getBatteryVoltage();
  }

  

  @Override
  public void periodic() {

    SmartDashboard.putNumber("IntakeRollerCurrent", m_intakeMotor.getStatorCurrent().getValueAsDouble()); 
    SmartDashboard.putBoolean("RollersHappy", m_intakeMotor.getStatorCurrent().getValueAsDouble()<25); 
    SmartDashboard.putNumber("UnhappyCount", SmartDashboard.getNumber("UnhappyCount", 0) + (SmartDashboard.getBoolean("ElevatorHappy", true) ? 0: 1));

  }
    
}
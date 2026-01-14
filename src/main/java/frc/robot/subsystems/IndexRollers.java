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
// constants is also missing intake_motor, so when we get that added in remove the comments mentioning it or anything related to it
// and of course if i need to change anything about this tell me

public class IndexRollers extends SubsystemBase {
  /** Creates a new IndexRollers. */
  private TalonFX m_indexMotor;
  DoubleLogEntry currentLog;
  BooleanLogEntry limitSwitchLog;

  public IndexRollers() {
    m_indexMotor = new TalonFX(Constants.INDEX_MOTOR.id, Constants.INDEX_MOTOR.busName);


    TalonFXConfiguration indexConfig = new TalonFXConfiguration();
    indexConfig.MotorOutput.Inverted =  InvertedValue.Clockwise_Positive;
    indexConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    indexConfig.MotorOutput.PeakForwardDutyCycle = 0.9;
    indexConfig.MotorOutput.PeakReverseDutyCycle = -0.9;
    indexConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    indexConfig.CurrentLimits.StatorCurrentLimit = 60;

    
    m_indexMotor.getConfigurator().apply(indexConfig);

  }

  double negative;
  public void wheelsIndex(double speed) {
    
    // redundant
    /*if (speed < 0) {
      negative = -1;
    } else {
      negative = 1;
    }*/

     if (speed < 0) {
       m_indexMotor.set(Math.min(speed, 0));
     } else {
       m_indexMotor.set(getPercentFromBattery(speed));
     }

  }
  public double getPercentFromBattery(double speed){
    return speed * 12 / RobotController.getBatteryVoltage();
  }

  

  @Override
  public void periodic() {
    SmartDashboard.putNumber("IndexRollerCurrent", m_indexMotor.getStatorCurrent().getValueAsDouble()); 
    SmartDashboard.putBoolean("RollersHappy", m_indexMotor.getStatorCurrent().getValueAsDouble()<25); 
    SmartDashboard.putNumber("UnhappyCount", SmartDashboard.getNumber("UnhappyCount", 0) + (SmartDashboard.getBoolean("ElevatorHappy", true) ? 0: 1));

  }
    
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Micro;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.math.estimator.SteadyStateKalmanFilter;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.state.BasicState;
import frc.robot.state.IDLE;
import frc.robot.state.INTAKE;
import frc.robot.state.INTAKE_OUT;
import frc.robot.state.INTAKE_WHILE_SHOOT;

public class StateManager extends SubsystemBase {

  Map<String, Object> desiredData = new HashMap<>();
  Map<String, Object> currentData = new HashMap<>();
  boolean isAtLocation = false;
  boolean willScore = true;

  public enum States {
    IDLE,
    INTAKE,
    INTAKE_OUT,
    INTAKE_WHILE_SHOOT,
    CLIMB_L1,
  }

  public List<BasicState> StateHandlers = List.of(
      new IDLE(),
      new INTAKE(),
      new INTAKE_OUT(),
      new INTAKE_WHILE_SHOOT(),
      new CLIMB_L1(),
  );

  public States state;

  private States previousState;

  /** Creates a new StateManager. */
  public StateManager() {
    

  }

  public void requestNewState(States newstate) {
    for (BasicState handler : StateHandlers) {
      if (handler.matches(newstate)) {
        handler.execute(this);
        isAtLocation = false;
        previousState = state;
        this.state = newstate;
      }
    }
  }

  public void clearCommandData() {

  }

  public Object getCurrentData(String key) {
    Object value = currentData.get(key);

    if (value instanceof String) {
      return (String) value;
    } else if (value instanceof Integer) {
      return (Integer) value;
    } else if (value instanceof Double) {
      return (Double) value;
    } else if (value instanceof Boolean) {
      return (Boolean) value;
    } else {
      return null; // Handle cases where the value is not of expected type
    }
  }

  

  public void returnToIdle(States state) {
    if (false) {
      if (state == States.IDLE) { //if state is in pickup algae state
        requestNewState(States.IDLE); //then go to algae idle state
      } else {
        requestNewState(States.IDLE); //preferably always idle
      }
    } else {
      requestNewState(States.IDLE);
    }
  }

  public void returnToIdle() {
    if ((boolean) currentData.get("IntakeLimitSwitch")) {
      if (state == States.IDLE) { //if state is in pickup algae state
        requestNewState(States.IDLE); //then go to algae idle state
      } else {
        requestNewState(States.IDLE); //preferably always idle
      }
    } else {
      requestNewState(States.IDLE);
    }
  }

  public void clearDesiredData(){
    desiredData.clear();
  }

  public Object getDesiredData(String key) {
    Object value = desiredData.get(key);

    if (value instanceof String) {
      return (String) value;
    } else if (value instanceof Integer) {
      return (Integer) value;
    } else if (value instanceof Double) {
      return (Double) value;
    } else if (value instanceof Boolean) {
      return (Boolean) value;
    } else {
      return null; // Handle cases where the value is not of expected type
    }
  }

  public void removeDesiredData(String key) {
    desiredData.remove(key);
  }

  public void addDesiredData(String key, Object object) {
    desiredData.put(key, object);
  }

  public void updateCurrentData(String key, Object object) {
    currentData.put(key, object);
  }

  public void setState(States state) {
    this.state = state;
  }

  public boolean getisAtLocaiton(){
    return isAtLocation;
  }


  @Override
  public void periodic() {

    for (Map.Entry<String, Object> entry : desiredData.entrySet()) {
      String key = entry.getKey();
      Object value = entry.getValue();

      if (value instanceof Double) {
        SmartDashboard.putNumber(key, (Double) value);
      }else if (value instanceof Boolean) {
        SmartDashboard.putBoolean(key, (Boolean) value);
      }else if (value instanceof Integer ){
        SmartDashboard.putNumber(key, (Integer) value);
      }
  }

    // This method will be called once per scheduler run
  }
}
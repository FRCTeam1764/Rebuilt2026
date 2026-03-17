package frc.robot.state;

import frc.robot.constants.CommandConstants;
import frc.robot.subsystems.StateManager;
import frc.robot.subsystems.StateManager.States;

public class CONDENSED implements BasicState { 
public boolean matches(States state){

    return state.equals(States.CONDENSED);
}

public void execute(StateManager stateManager){
    double tempIntake = (double)stateManager.getCurrentData(CommandConstants.INTAKE_WRIST_KEY);

    stateManager.clearDesiredData();

    stateManager.addDesiredData(CommandConstants.INTAKE_WRIST_KEY, CommandConstants.INTAKE_WRIST_IN);
    stateManager.addDesiredData(CommandConstants.SHOOTER_FLYWHEEL_KEY, 0.0);
    stateManager.addDesiredData(CommandConstants.INTAKE_KEY, 0.0);
    stateManager.addDesiredData(CommandConstants.INDEX_KEY, 0.0);
    stateManager.addDesiredData(CommandConstants.INTAKE_WRIST_KEY, tempIntake);
}
   
}

package frc.robot.state;

import frc.robot.constants.CommandConstants;
import frc.robot.subsystems.StateManager;
import frc.robot.subsystems.StateManager.States;

public class MID_SHOOT implements BasicState {
public boolean matches(States state){

    return state.equals(States.SHOOT);
}

public void execute(StateManager stateManager){
    stateManager.clearDesiredData();

    stateManager.addDesiredData(CommandConstants.SHOOTER_FLYWHEEL_KEY, CommandConstants.SHOOTER_SPEED);
    stateManager.addDesiredData(CommandConstants.RES_INDEX_KEY, CommandConstants.RES_SPEED);
    stateManager.addDesiredData(CommandConstants.INTAKE_WRIST_KEY, CommandConstants.INTAKE_WRIST_MID);
    stateManager.addDesiredData(CommandConstants.INTAKE_KEY, 0.0);
    stateManager.addDesiredData(CommandConstants.INDEX_KEY, CommandConstants.INDEX_SPEED);
}
   
}

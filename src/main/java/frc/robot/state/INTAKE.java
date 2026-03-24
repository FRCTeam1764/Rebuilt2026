package frc.robot.state;

import frc.robot.constants.CommandConstants;
import frc.robot.subsystems.StateManager;
import frc.robot.subsystems.StateManager.States;

public class INTAKE implements BasicState {
public boolean matches(States state){

    return state.equals(States.INTAKE);
}

public void execute(StateManager stateManager){
    stateManager.clearDesiredData();

    stateManager.addDesiredData(CommandConstants.SHOOTER_FLYWHEEL_KEY, 0.0);
    stateManager.addDesiredData(CommandConstants.RES_INDEX_KEY, 0.0);
    stateManager.addDesiredData(CommandConstants.INTAKE_KEY, CommandConstants.INTAKE_IN_SPEED);
    stateManager.addDesiredData(CommandConstants.INDEX_KEY, 0.0);
    stateManager.addDesiredData(CommandConstants.INTAKE_WRIST_KEY, CommandConstants.INTAKE_WRIST_DOWN);
}
   
}

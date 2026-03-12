package frc.robot.state;

import frc.robot.constants.CommandConstants;
import frc.robot.subsystems.StateManager;
import frc.robot.subsystems.StateManager.States;

public class IDLE implements BasicState {
public boolean matches(States state){

    return state.equals(States.IDLE);
}

public void execute(StateManager stateManager){
    stateManager.clearDesiredData();


    stateManager.addDesiredData(CommandConstants.SHOOTER_ROLLER_KEY, 0.0);
    stateManager.addDesiredData(CommandConstants.INTAKE_WRIST_KEY, 0.0);
    stateManager.addDesiredData(CommandConstants.INTAKE_KEY, 0.0);
    stateManager.addDesiredData(CommandConstants.INDEX_KEY, 0.0);
    stateManager.addDesiredData(CommandConstants.CLIMBER_KEY,0.0);
}
   
}

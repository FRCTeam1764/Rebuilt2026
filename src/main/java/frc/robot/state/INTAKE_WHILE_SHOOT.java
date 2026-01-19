package frc.robot.state;

import frc.robot.constants.CommandConstants;
import frc.robot.subsystems.StateManager;
import frc.robot.subsystems.StateManager.States;

public class IDLE implements BasicState {
public boolean matches(States state){

    return state.equals(States.INTAKE_WHILE_SHOOT);
}

public void execute(StateManager stateManager){
    stateManager.clearDesiredData();

    stateManager.addDesiredData(CommandConstants.TURRET_KEY, 0.0);
    stateManager.addDesiredData(CommandConstants.WRIST_KEY, 0.0);
    stateManager.addDesiredData(CommandConstants.SHOOTER_ROLLER_KEY, 0.5);
    stateManager.addDesiredData(CommandConstants.INTAKE_ROLLER_KEY, 0.5);
    stateManager.addDesiredData(CommandConstants.INDEX_ROLLER_KEY, 0.5);
}
   
}

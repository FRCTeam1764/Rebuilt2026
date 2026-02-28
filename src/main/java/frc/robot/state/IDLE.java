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

    // if something's messed up as a result of including the other components in this state
    // tell elliott
    stateManager.addDesiredData(CommandConstants.TURRET_KEY, 0.0);
    stateManager.addDesiredData(CommandConstants.WRIST_KEY, 0.0);
    stateManager.addDesiredData(CommandConstants.SHOOTER_ROLLER_KEY, 0.0);
    stateManager.addDesiredData(CommandConstants.INTAKE_KEY, 0.0); // originally 30.0
    stateManager.addDesiredData(CommandConstants.INDEX_KEY, 0.0);
    stateManager.addDesiredData(CommandConstants.CLIMBER_KEY, 0.0);
}
   
}

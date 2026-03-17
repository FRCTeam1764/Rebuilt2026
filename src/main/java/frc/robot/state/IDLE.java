package frc.robot.state;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.CommandConstants;
import frc.robot.subsystems.StateManager;
import frc.robot.subsystems.StateManager.States;

public class IDLE implements BasicState {
public boolean matches(States state){

    return state.equals(States.IDLE);
}

public void execute(StateManager stateManager){
    double tempIntake = (double)stateManager.getCurrentData(CommandConstants.INTAKE_WRIST_KEY);
    
    stateManager.clearDesiredData();

    stateManager.addDesiredData(CommandConstants.SHOOTER_FLYWHEEL_KEY, 0.0);
    stateManager.addDesiredData(CommandConstants.INTAKE_KEY, 0.0);
    stateManager.addDesiredData(CommandConstants.RES_INDEX_KEY, 0.0);
    stateManager.addDesiredData(CommandConstants.INDEX_KEY, 0.0);
    stateManager.addDesiredData(CommandConstants.INTAKE_WRIST_KEY, tempIntake);
}
   
}

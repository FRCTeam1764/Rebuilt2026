package frc.robot.state;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.CommandConstants;
import frc.robot.subsystems.StateManager;
import frc.robot.subsystems.StateManager.States;

public class CLIMB_L1 implements BasicState {
public boolean matches(States state){

    return state.equals(States.CLIMB_L1);
}

public void execute(StateManager stateManager){
    stateManager.clearDesiredData();

    stateManager.addDesiredData(CommandConstants.SHOOTER_FLYWHEEL_KEY, 0.0);
    stateManager.addDesiredData(CommandConstants.INTAKE_WRIST_KEY, 0.0);
    stateManager.addDesiredData(CommandConstants.INTAKE_KEY, 0.0);
    stateManager.addDesiredData(CommandConstants.INDEX_KEY, 0.0);
}
   
}

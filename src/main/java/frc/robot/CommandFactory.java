// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.waitUntilPosition;
import frc.robot.commands.waitUntilPositionIndex;
import frc.robot.commands.BasicCommands.RequestStateChange;
import frc.robot.commands.ComplexCommands.returnToIdle;
import frc.robot.commands.DriveCommands.DriveForward;
import frc.robot.commands.DriveCommands.DriveToTarget;
import frc.robot.commands.DriveCommands.DriveToTargetOffset;
import frc.robot.commands.DriveCommands.DriveToTargetOffsetLL3;
import frc.robot.commands.DriveCommands.LockOnAprilTag;
import frc.robot.commands.DriveCommands.TurnToAngle;
import frc.robot.constants.CommandConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.StateManager;
import frc.robot.subsystems.StateManager.States;

//This class will handle all command handling for drivers

/** Add your docs here. */

public class CommandFactory {

    private CommandSwerveDrivetrain swerve;
    private StateManager stateManager;
    private CommandXboxController pilot;

    public CommandFactory(CommandXboxController pilot, CommandSwerveDrivetrain swerve, StateManager stateManager) {
        this.pilot = pilot;
        this.swerve = swerve;
        this.stateManager = stateManager;
    }

    public Command driveForward() {
        return new ParallelDeadlineGroup(
                    new WaitCommand(0.15), 
                    new DriveForward(swerve));
    }

    public Command ClimbL1Command(){
        return new ParallelDeadlineGroup(null, null)
    }
// we need climb command :/

    public Command HubShootCommand(){
        return new ParallelDeadlineGroup(
            new 
        )
    }

    public Command interupted(boolean wasInteruppted) {
        if (wasInteruppted) {
            return new InstantCommand();
        }
        return new returnToIdle(stateManager);
    }

    //desiredAction might not be used
    public enum desiredAction {
        IDLE
    }
    public desiredAction currentAction = desiredAction.IDLE;

} 

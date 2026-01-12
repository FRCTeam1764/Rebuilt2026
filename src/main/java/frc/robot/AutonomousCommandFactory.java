// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.waitUntilPosition;
import frc.robot.commands.BasicCommands.RequestStateChange;
import frc.robot.commands.ComplexCommands.returnToIdle;
import frc.robot.commands.DriveCommands.DriveBackward;
import frc.robot.commands.DriveCommands.DriveForward;
import frc.robot.commands.DriveCommands.DriveToTargetOffset;
import frc.robot.commands.DriveCommands.LockOnAprilTagAuto;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.StateManager;
import frc.robot.subsystems.StateManager.States;

/** Add your docs here. */
public class AutonomousCommandFactory extends CommandFactory{
    private CommandXboxController pilot;
    private CommandSwerveDrivetrain swerve;
    private StateManager stateManager;

    public AutonomousCommandFactory(CommandXboxController pilot, CommandSwerveDrivetrain swerve, StateManager stateManager) {
        super(pilot, swerve, stateManager);
        configAutonomousCommands();
    }

    


    public void configAutonomousCommands() {
        NamedCommands.registerCommand("idle", new RequestStateChange(States.IDLE, stateManager));
    }

}

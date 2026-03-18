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
import frc.robot.commands.BasicCommands.IntakeWristCommand;
import frc.robot.commands.BasicCommands.RequestStateChange;
import frc.robot.commands.ComplexCommands.returnToIdle;
import frc.robot.commands.DriveCommands.DriveBackward;
import frc.robot.commands.DriveCommands.DriveForward;
import frc.robot.commands.LimelightCommands.LockOnAprilTagAuto;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeWristRev;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterWristRev;
import frc.robot.subsystems.StateManager;
import frc.robot.subsystems.StateManager.States;
import frc.robot.subsystems.TurretRev;
import frc.robot.subsystems.RollersSubsystem;


/** Add your docs here. */
public class AutonomousCommandFactory extends CommandFactory{
    private CommandXboxController pilot;
    private CommandSwerveDrivetrain swerve;
    private StateManager stateManager;
    private RollersSubsystem rollers;
    private LimelightSubsystem limelight1;
    private LimelightSubsystem limelight2;
    private ShooterWristRev wrist;
    private TurretRev turret;
    private IntakeWristRev intakeWrist;
    private ClimberSubsystem climber;

    public AutonomousCommandFactory(IntakeWristRev intakeWrist, TurretRev turret, ShooterWristRev wrist, LimelightSubsystem turretLimelight, 
                RollersSubsystem rollers, ClimberSubsystem climber, 
                CommandXboxController pilot, CommandSwerveDrivetrain swerve, StateManager stateManager) {
        super(intakeWrist, turret, wrist, turretLimelight, rollers, climber, pilot, swerve, stateManager);
        configAutonomousCommands();
    }
    
    // public Command HubShootCommand_Q1(){
    //     return new SequentialCommandGroup( new RequestStateChange(States.SHOOT_Q1, stateManager),
    //     new ParallelCommandGroup(
    //         new AimTurretAtHub(swerve, pilot, localization)
    //         //new AimWristCommand()
    //     )
    //     );
    // }

    // public Command HubShootCommand_Q2(){
    //     return new SequentialCommandGroup( new RequestStateChange(States.SHOOT_Q2, stateManager),
    //     new ParallelCommandGroup(
    //         new AimTurretAtHub(swerve, pilot, localization)
    //         //new AimWristCommand()
    //     )
    //     );
    // }

    // public Command HubShootCommand_Q3(){
    //     return new SequentialCommandGroup( new RequestStateChange(States.SHOOT_Q3, stateManager),
    //     new ParallelCommandGroup(
    //         new AimTurretAtHub(swerve, pilot, localization)
    //         //new AimWristCommand()
    //     )
    //     );
    // }

    // public Command HubShootCommand_Q4(){
    //     return new SequentialCommandGroup( new RequestStateChange(States.SHOOT_Q4, stateManager),
    //     new ParallelCommandGroup(
    //         new AimTurretAtHub(swerve, pilot, localization)
    //         //new AimWristCommand()
    //     )
    //     );
    // }

    // public Command HubShootCommand_Q5(){
    //     return new SequentialCommandGroup( new RequestStateChange(States.SHOOT_Q5, stateManager),
    //     new ParallelCommandGroup(
    //         new AimTurretAtHub(swerve, pilot, localization)
    //         //new AimWristCommand()
    //     )
    //     );
    // }

    // public Command HubShootCommand_Q6(){
    //     return new SequentialCommandGroup( new RequestStateChange(States.SHOOT_Q6, stateManager),
    //     new ParallelCommandGroup(
    //         new AimTurretAtHub(swerve, pilot, localization)
    //         //new AimWristCommand()
    //     )
    //     );
    // }

    // public Command HubShootCommand_Q7(){
    //     return new SequentialCommandGroup( new RequestStateChange(States.SHOOT_Q7, stateManager),
    //     new ParallelCommandGroup(
    //         new AimTurretAtHub(swerve, pilot, localization)
    //         //new AimWristCommand()
    //     )
    //     );
    // }

    // public Command HubShootCommand_Q8(){
    //     return new SequentialCommandGroup( new RequestStateChange(States.SHOOT_Q8, stateManager),
    //     new ParallelCommandGroup(
    //         new AimTurretAtHub(swerve, pilot, localization)
    //         //new AimWristCommand()
    //     )
    //     );
    // }

    // arctan(2 + ((-9.8 * x^2)/(2(9.1^2)))

    //we need to add ShootTop and ShootBottom for cycling fuel to our zone in autos (just a set angle each)
    
    public Command DepotIntake(){
        return new SequentialCommandGroup(
        new ParallelDeadlineGroup(
            new WaitCommand(0.2),
            new DriveForward(swerve),
            GroundIntakeCommand()
        ),
        new RequestStateChange(States.IDLE, stateManager)
        );
    }

// find out how many seconds each one should be (depot should be easy, neutral will be based on capacity)

    public Command NeutralIntake(){
        return new SequentialCommandGroup( 
        new ParallelDeadlineGroup(
            new WaitCommand(0.4),
            new DriveForward(swerve),
            GroundIntakeCommand()
        ),
        new RequestStateChange(States.IDLE, stateManager)
        );
    }

    public void configAutonomousCommands() {
        NamedCommands.registerCommand("Q1turret", null);
        NamedCommands.registerCommand("GroundIntakeCommand", GroundIntakeCommand());
        NamedCommands.registerCommand("ClimbCommand", ClimbUpCommand());
        NamedCommands.registerCommand("DriveForward", new DriveForward(swerve));
        NamedCommands.registerCommand("idle", new RequestStateChange(States.IDLE, stateManager));
    }

}

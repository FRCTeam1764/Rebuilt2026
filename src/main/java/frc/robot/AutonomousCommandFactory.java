// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import javax.sql.StatementEvent;

import com.fasterxml.jackson.core.json.WriterBasedJsonGenerator;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.BasicCommands.IntakeWristCommand;
import frc.robot.commands.BasicCommands.ShooterWristCommand;
import frc.robot.commands.DriveCommands.DriveBackward;
import frc.robot.commands.DriveCommands.DriveForward;
import frc.robot.commands.LimelightCommands.LockOnAprilTagAuto;
import frc.robot.constants.CommandConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeWristRev;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterWristRev;
import frc.robot.subsystems.TurretRev;
import frc.robot.subsystems.RollersSubsystem;


/** Add your docs here. */
public class AutonomousCommandFactory extends CommandFactory{
    private CommandXboxController pilot;
    private CommandSwerveDrivetrain swerve;
    private RollersSubsystem rollers;
    private LimelightSubsystem limelight1;
    private LimelightSubsystem limelight2;
    private ShooterWristRev shooterWrist;
    private TurretRev turret;
    private IntakeWristRev intakeWrist;

    public AutonomousCommandFactory(IntakeWristRev intakeWrist, TurretRev turret, ShooterWristRev wrist, LimelightSubsystem turretLimelight, 
                RollersSubsystem rollers,
                CommandXboxController pilot, CommandSwerveDrivetrain swerve) {
        super(intakeWrist, turret, wrist, turretLimelight, rollers, pilot, swerve);
        this.shooterWrist = wrist;
        this.swerve = swerve;
        this.intakeWrist = intakeWrist;
        this.turret = turret;
        this.rollers = rollers;
        this.limelight1 = turretLimelight;
        this.pilot = pilot;
        configAutonomousCommands();
    }
    

    // arctan(2 + ((-9.8 * x^2)/(2(9.1^2)))

    //we need to add ShootTop and ShootBottom for cycling fuel to our zone in autos (just a set angle each)
    
    

// find out how many seconds each one should be (depot should be easy, neutral will be based on capacity)

    

    public void configAutonomousCommands() {
        NamedCommands.registerCommand("R1Wrist", new ShooterWristCommand(CommandConstants.R1_SHOOTER, shooterWrist));
        NamedCommands.registerCommand("R2Wrist", new ShooterWristCommand(CommandConstants.R2_SHOOTER, shooterWrist));
        NamedCommands.registerCommand("R3Wrist", new ShooterWristCommand(CommandConstants.R3_SHOOTER, shooterWrist));
        NamedCommands.registerCommand("R4Wrist", new ShooterWristCommand(CommandConstants.R4_SHOOTER, shooterWrist));
        NamedCommands.registerCommand("WristDown", new ShooterWristCommand(CommandConstants.RDOWN_SHOOTER, shooterWrist));
        //NamedCommands.registerCommand("GroundIntakeCommand", GroundIntakeCommand());
        NamedCommands.registerCommand("HubShootCommand", ShootRampCommand());
        NamedCommands.registerCommand("GroundIntake", GroundIntakeCommand());
        NamedCommands.registerCommand("Unjam", unJamSpin());
        // NamedCommands.registerCommand("WaitUntilIdle", new SequentialCommandGroup(
        //     new RequestStateChange(States.IDLE, stateManager),
        //     new waitUntilPosition(stateManager, CommandConstants.INTAKE_WRIST_KEY, 0.5)));
        //.registerCommand("null", null);

        //NamedCommands.registerCommand("ClimbCommand", ClimbUpCommand());
        NamedCommands.registerCommand("DriveForward", new DriveForward(swerve));
        NamedCommands.registerCommand("idle", resetPos());
    }

}

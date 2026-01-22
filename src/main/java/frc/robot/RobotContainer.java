// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.concurrent.CopyOnWriteArrayList;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.subsystems.StateManager;
import frc.robot.subsystems.StateManager.States;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterRollers;
import frc.robot.subsystems.ShooterWrist;
import frc.robot.commands.BasicCommands.RequestStateChange;
import frc.robot.commands.DefaultCommands.DefaultIndexCommand;
import frc.robot.commands.DefaultCommands.DefaultIntakeCommand;
import frc.robot.commands.DefaultCommands.DefaultShooterRollersCommand;
import frc.robot.commands.DefaultCommands.DefaultShooterWristCommand;
import frc.robot.commands.DefaultCommands.DefaultTurretCommand;
import frc.robot.commands.DriveCommands.DriveRobotCentric;
import frc.robot.commands.DriveCommands.DriveToLimeLightVisionOffset;
import frc.robot.commands.DriveCommands.DriveToTargetOffset;
import frc.robot.commands.DriveCommands.DriveToTargetOffsetLL3;
import frc.robot.commands.DriveCommands.LockOnAprilTag;
import frc.robot.commands.DriveCommands.TrackObject;
import frc.robot.commands.DriveCommands.TurnToAngle;
import frc.robot.constants.CommandConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.state.IDLE;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IndexRollers;
import frc.robot.subsystems.IntakeRollers;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.06).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric().withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.025).withDriveRequestType(DriveRequestType.OpenLoopVoltage).withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController pilot = new CommandXboxController(0);
    private final CommandXboxController copilot = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final StateManager stateManager = new StateManager();
    
    //subsystems
    private final ShooterRollers shootRollers = new ShooterRollers();
    private final IndexRollers indexRollers = new IndexRollers();
    private final IntakeRollers intakeRollers = new IntakeRollers();
    private final ShooterWrist wrist = new ShooterWrist(stateManager);
    private final Turret turret = new Turret();
    
    //limelights
    private final LimelightSubsystem turretLimelight = new LimelightSubsystem("limelight");
    private final LimelightSubsystem localLimelight = new LimelightSubsystem("limelight3");
    
    //factories
    private final CommandFactory commandFactory = new CommandFactory(turret, wrist, turretLimelight, localLimelight, intakeRollers, indexRollers, shootRollers, pilot, drivetrain, stateManager);
    private final AutonomousCommandFactory autoFactory = new AutonomousCommandFactory(turret, wrist, turretLimelight, localLimelight, intakeRollers, indexRollers, shootRollers, pilot, drivetrain, stateManager);

    private final SendableChooser<Command> chooser ;

    public RobotContainer() {
        stateManager.requestNewState(States.IDLE);
        chooser = AutoBuilder.buildAutoChooser("Autonomous");
        SmartDashboard.putData("Autos", chooser);
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-pilot.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-pilot.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-pilot.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Default Commands
        shootRollers.setDefaultCommand(new DefaultShooterRollersCommand(shootRollers, stateManager));
        indexRollers.setDefaultCommand(new DefaultIndexCommand(indexRollers, stateManager));
        intakeRollers.setDefaultCommand(new DefaultIntakeCommand(intakeRollers, stateManager));
        wrist.setDefaultCommand(new DefaultShooterWristCommand(wrist, stateManager));
        turret.setDefaultCommand(new DefaultTurretCommand(turret, stateManager));

        configureMainBindings();

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private void configureMainBindings() {
        // Drive Controls
        pilot.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        pilot.start().onTrue(new RequestStateChange(States.IDLE, stateManager));
        copilot.leftTrigger(.7).whileTrue(drivetrain.applyRequest(()->brake));
        pilot.b().whileTrue(new DriveRobotCentric(drivetrain, pilot));

        // Subsystem Controls
        
        // Limelight Controls
        
    }

    public void changePipeline() {
        localLimelight.setPipeline(1);
    }

    public Command getAutonomousCommand() {
        return chooser.getSelected();
    }
}

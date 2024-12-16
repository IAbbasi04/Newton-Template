// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Controls.ControlSets;
import frc.robot.autonomous.AutoManager;
import frc.robot.commands.*;
import frc.robot.commands.proxies.NewtonWrapperCommand;
import frc.robot.subsystems.*;
import frc.robot.subsystems.SwerveSubsystem.DriveModes;
import lib.team8592.MatchMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;

public class RobotContainer {
    private SubsystemManager activeSubsystemsManager;
    private SwerveSubsystem swerve;

    private boolean logToShuffleboard = false;

    /**
     * Create the robot container. This creates and configures subsystems, sets
     * up button bindings, and prepares for autonomous.
     */
    public RobotContainer(boolean logToShuffleboard) {
        this.activeSubsystemsManager = new SubsystemManager(logToShuffleboard);
        this.logToShuffleboard = logToShuffleboard;
        
        NewtonCommands.initialize(activeSubsystemsManager);
        NewtonWrapperCommand.initialize(activeSubsystemsManager);
        AutoManager.prepare(activeSubsystemsManager);

        Controls.initializeShuffleboardLogs(logToShuffleboard);

        // Add subsystems here
        swerve = activeSubsystemsManager.getSwerve();

        this.configureBindings(ControlSets.DUAL_DRIVER);
        this.configureDefaults();
        this.registerNamedCommands();
    }

    /**
     * Registers all named commands to be used as events in PathPlanner autos
     */
    private void registerNamedCommands() {
        // Register named commands for PathPlanner event markers
    }

    /**
     * Configure default commands for the subsystems
     */
    private void configureDefaults(){
        // Set the swerve's default command to drive with joysticks
        swerve.setDefaultCommand(swerve.run(() -> {
            swerve.drive(swerve.processJoystickInputs(
                Controls.driveTranslateX.getAsDouble(),
                Controls.driveTranslateY.getAsDouble(),
                Controls.driveRotate.getAsDouble()
            ), DriveModes.AUTOMATIC);
        }));
    }

    /**
     * Configure all button bindings
     *
     * @param controlSet the set of controls to use
     */
    private void configureBindings(ControlSets controlSet) {
        CommandScheduler.getInstance().getDefaultButtonLoop().clear();
        Controls.applyControlSet(controlSet);

        Controls.slowMode.onTrue(
            Commands.runOnce(() -> swerve.setSlowMode(true)).ignoringDisable(true)
        ).onFalse(
            Commands.runOnce(() -> swerve.setSlowMode(false)).ignoringDisable(true)
        );

        Controls.zeroGryoscope.onTrue(
            // Similar comment on Commands.runOnce as slow mode above
            Commands.runOnce(() -> swerve.resetHeading())
        );

        Controls.robotRelative.onTrue(
            // Similar comment on Commands.runOnce and ignoringDisable as slow mode above
            Commands.runOnce(() -> swerve.setRobotRelative(true)).ignoringDisable(true)
        ).onFalse(
            Commands.runOnce(() -> swerve.setRobotRelative(false)).ignoringDisable(true)
        );

        Controls.snapForward.whileTrue(
            NewtonCommands.swerveSnapToCommand(
                Rotation2d.fromDegrees(0),
                Controls.driveTranslateX,
                Controls.driveTranslateY
            )
        );

        Controls.snapBack.whileTrue(
            NewtonCommands.swerveSnapToCommand(
                Rotation2d.fromDegrees(180),
                Controls.driveTranslateX,
                Controls.driveTranslateY
            )
        );

        Controls.snapRight.whileTrue(
            NewtonCommands.swerveSnapToCommand(
                Rotation2d.fromDegrees(270),
                Controls.driveTranslateX,
                Controls.driveTranslateY
            )
        );

        Controls.snapLeft.whileTrue(
            NewtonCommands.swerveSnapToCommand(
                Rotation2d.fromDegrees(90),
                Controls.driveTranslateX,
                Controls.driveTranslateY
            )
        );
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return AutoManager.getAutonomousCommand();
    }

    /**
     * Whether we want to log to shuffleboard
     */
    public boolean logToShuffleboard() {
        return logToShuffleboard;
    }

    /**
     * Runs the onInit() method for each active subsystem based on the given mode
     */
    public void runSubsystemsInit(MatchMode mode) {
        activeSubsystemsManager.onInit(mode);
    }
}

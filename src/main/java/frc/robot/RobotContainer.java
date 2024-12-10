// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Controls.ControlSets;
import frc.robot.autonomous.AutoManager;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.SwerveSubsystem.DriveModes;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;

public class RobotContainer {
    // The robot's subsystems
    public static SwerveSubsystem swerve;

    private boolean logToShuffleboard = false;

    /**
     * Create the robot container. This creates and configures subsystems, sets
     * up button bindings, and prepares for autonomous.
     */
    public RobotContainer(boolean logToShuffleboard) {
        RobotContainer.swerve = new SwerveSubsystem(logToShuffleboard).initializeAutoBuilder();

        this.logToShuffleboard = logToShuffleboard;

        this.configureBindings(ControlSets.DUAL_DRIVER);
        this.configureDefaults();
        this.registerNamedCommands();

        Controls.initializeShuffleboardLogs(logToShuffleboard);
        AutoManager.prepare();
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
        setDefaultCommand(swerve, swerve.run(() -> {
            swerve.drive(swerve.processJoystickInputs(
                Controls.driveTranslateX.getAsDouble(),
                Controls.driveTranslateY.getAsDouble(),
                Controls.driveRotate.getAsDouble()
            ), DriveModes.AUTOMATIC);
        }).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    }

    //Any commands that are reused a lot but can't go in a separate class go here

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
            Commands.runOnce(() -> swerve.setRobotRelative(true)).ignoringDisable(true)
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
     * Set the default command of a subsystem (what to run if no other command requiring it is running).
     * <p> NOTE: all subsystems also have a setDefaultCommand method; this version includes a check for
     * default commands that cancel incoming commands that require the subsystem. Unless you're sure
     * of what you're doing, you should use this one.
     *
     * @param subsystem the subsystem to apply the default command to
     * @param command to command to set as default
     */
    private void setDefaultCommand(SubsystemBase subsystem, Command command){
        if(command.getInterruptionBehavior() == InterruptionBehavior.kCancelSelf){
            subsystem.setDefaultCommand(command);
        }
        else{
            //If you want to force-allow setting a cancel-incoming default command, directly call `subsystem.setDefaultCommand()` instead
            throw new UnsupportedOperationException("Can't set a default command that cancels incoming!");
        }
    }

    /**
     * Whether we want to log to shuffleboard
     */
    public boolean logToShuffleboard() {
        return logToShuffleboard;
    }
}

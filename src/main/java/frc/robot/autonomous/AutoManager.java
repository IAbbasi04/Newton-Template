// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.robot.autonomous.autos.DefaultAuto;
import frc.robot.commands.proxies.*;
import frc.robot.subsystems.SubsystemManager;

/**
 * General class for autonomous management (loading autos, sending the chooser, getting the
 * user-selected auto command, etc).
 */
public final class AutoManager {
    private static SendableChooser<AutoCommand> autoChooser;
    private static ArrayList<AutoCommand> autoCommands = new ArrayList<>();
    private static SubsystemManager manager;

    /**
     * Load all autos and broadcast the chooser.
     *<p>
     * * This is where programmers should add new autos.
     *
     * @apiNote This should be called on {@link Robot#robotInit()} only;
     * this function will have relatively long delays due to loading paths.
     */
    public static void prepare(SubsystemManager manager){
        AutoManager.autoCommands = new ArrayList<>();
        AutoManager.manager = manager;

        // autoCommands.add(new ExampleAuto());
        // TODO: Add autos here like example above

        AutoManager.autoChooser = new SendableChooser<>();
        AutoManager.autoChooser.setDefaultOption("DEFAULT - DO NOTHING", new DefaultAuto());
        for(AutoCommand c : autoCommands){
            autoChooser.addOption(
                c.getAutoName()+(
                    c.getStartPose() == null ? " (WARNING: NO START POSE)" : ""
                ), c
            );
        }
        Shuffleboard.getTab("Autonomous Config").add(autoChooser);
    }

    /**
     * Get the user-selected autonomous command as determined by {@link AutoManager#autoChooser}
     *
     * @return the command
     */
    public static Command getAutonomousCommand(){
        AutoCommand autoCommand = autoChooser.getSelected();

        return manager.onAutonomousInitCommand().andThen(
            // If we don't keep this command from registering as composed,
            // the code will crash if we try to run an auto twice without
            // restarting robot code.
            new MultiComposableCommand(autoCommand)
        );
    }

    private AutoManager() {
        throw new UnsupportedOperationException("This is a utility class!");
    }
}

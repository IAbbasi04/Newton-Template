package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj2.command.*;
import lib.team8592.MatchMode;

public class SubsystemManager {
    private SwerveSubsystem swerveSubsystem;
    private IntakeSubsystem intakeSubsystem;

    private List<NewtonSubsystem> activeSubystems = new ArrayList<>();

    public SubsystemManager(boolean logToShuffleboard) {
        this.swerveSubsystem = new SwerveSubsystem(logToShuffleboard);
        this.intakeSubsystem = new IntakeSubsystem(logToShuffleboard);

        this.activeSubystems = List.of(
            // Add all active subsystems here
            swerveSubsystem,
            intakeSubsystem
        );

        this.activeSubystems.forEach(s -> {
            s.enableSubsystem(true);
            s.initializeLogger();
        });
    }

    public Command onAutonomousInitCommand() {
        NewtonSubsystem[] subs = new NewtonSubsystem[activeSubystems.size()];
        for (int i = 0; i < activeSubystems.size(); i++) {
            subs[i] = activeSubystems.get(i);
        }

        return Commands.runOnce(() -> {
            activeSubystems.forEach(s -> s.onInit(MatchMode.AUTONOMOUS));
        }, subs);
    }

    public Command onInitCommand(MatchMode mode) {
        NewtonSubsystem[] subs = new NewtonSubsystem[activeSubystems.size()];
        for (int i = 0; i < activeSubystems.size(); i++) {
            subs[i] = activeSubystems.get(i);
        }

        return Commands.runOnce(() -> {
            activeSubystems.forEach(s -> s.onInit(mode));
        }, subs);
    }

    // ==================================== \\
    // Add all getSubsystem() methods below \\
    // ==================================== \\

    public SwerveSubsystem getSwerve() {
        return this.swerveSubsystem;
    }

    public IntakeSubsystem getIntake() {
        return this.intakeSubsystem;
    }
}
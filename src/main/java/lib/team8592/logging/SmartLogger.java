package lib.team8592.logging;

import java.util.Dictionary;
import java.util.Hashtable;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.*;

public class SmartLogger {
    private String n_name = "";
    private ShuffleboardTab shuffleboardTab;
    private Dictionary<String, GenericEntry> cards;

    private boolean logToShuffleboard = true;

    public SmartLogger(String name, boolean logToShuffleboard) {
        this.n_name = name;
        this.logToShuffleboard = logToShuffleboard;

        if (!logToShuffleboard) return;

        this.shuffleboardTab = Shuffleboard.getTab(name);
        this.cards = new Hashtable<>();
    }

    public SmartLogger(String name) {
        this.n_name = name;
        this.shuffleboardTab = Shuffleboard.getTab(name);

        this.cards = new Hashtable<>();
    }

    public void enable() {
        this.logToShuffleboard = true;
    }

    public void disable() {
        this.logToShuffleboard = false;
    }

    public void logDouble(String key, double value) {
        Logger.recordOutput(n_name + "/" + key, value); // Record to AdvantageKit logs
        if (!this.logToShuffleboard) return; // Do not proceed if we do not want to log to shuffleboard
        if (cards.get(key) == null) { // Card doesn't exist yet on shuffleboard
            cards.put(key, shuffleboardTab.add(key, value).getEntry());
        } else { // Card already exists on shuffleboard
            cards.get(key).setDouble(value);
        }
    }

    public void logString(String key, String value) {
        Logger.recordOutput(n_name + "/" + key, value); // Record to AdvantageKit logs
        if (!this.logToShuffleboard) return; // Do not proceed if we do not want to log to shuffleboard
        if (cards.get(key) == null) { // Card doesn't exist yet on shuffleboard
            cards.put(key, shuffleboardTab.add(key, value).getEntry());
        } else { // Card already exists on shuffleboard
            cards.get(key).setString(value);
        }
    }

    public void logBoolean(String key, boolean value) {
        Logger.recordOutput(n_name + "/" + key, value); // Record to AdvantageKit logs
        if (!this.logToShuffleboard) return; // Do not proceed if we do not want to log to shuffleboard
        if (cards.get(key) == null) { // Card doesn't exist yet on shuffleboard
            cards.put(key, shuffleboardTab.add(key, value).getEntry());
        } else { // Card already exists on shuffleboard
            cards.get(key).setBoolean(value);
        }
    }

    public <E extends Enum<E>> void logEnum(String key, E value) {
        Logger.recordOutput(n_name + "/" + key, value.name()); // Record to AdvantageKit logs
        if (!this.logToShuffleboard) return; // Do not proceed if we do not want to log to shuffleboard
        if (cards.get(key) == null) { // Card doesn't exist yet on shuffleboard
            cards.put(key, shuffleboardTab.add(key, value.name()).getEntry());
        } else { // Card already exists on shuffleboard
            cards.get(key).setString(value.name());
        }
    }

    public void logChassisSpeeds(String key, ChassisSpeeds value) {
        Logger.recordOutput(n_name + "/" + key, value); // Record to AdvantageKit logs
        if (!this.logToShuffleboard) return; // Do not proceed if we do not want to log to shuffleboard
        if (cards.get(key) == null) { // Card doesn't exist yet on shuffleboard
            cards.put(key, shuffleboardTab.add(key, value.toString()).getEntry());
        } else { // Card already exists on shuffleboard
            cards.get(key).setString(value.toString());
        }
    }

    public void logPose2d(String key, Pose2d value) {
        Logger.recordOutput(n_name + "/" + key, value); // Record to AdvantageKit logs
        if (!this.logToShuffleboard) return; // Do not proceed if we do not want to log to shuffleboard
        if (cards.get(key) == null) { // Card doesn't exist yet on shuffleboard
            cards.put(key, shuffleboardTab.add(key, value.toString()).getEntry());
        } else { // Card already exists on shuffleboard
            cards.get(key).setString(value.toString());
        }
    }
}
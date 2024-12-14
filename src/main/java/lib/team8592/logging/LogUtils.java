package lib.team8592.logging;

import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Suppliers;

public class LogUtils {
    public static class LogConstants {
        public final String game, year, robot, team;
        public LogConstants(String game, String year, String robot, String team) {
            this.game = game;
            this.year = year;
            this.robot = robot;
            this.team = team;
        }
    }
    
    public static void initialize(LogConstants constants) {
        Logger.recordMetadata("Game", constants.game);
        Logger.recordMetadata("Year", constants.year);
        Logger.recordMetadata("Robot", constants.robot);
        Logger.recordMetadata("Team", constants.team);

        if (Suppliers.robotIsReal.getAsBoolean()) { // If running on a real robot
            String time = DateTimeFormatter.ofPattern("yy-MM-dd_HH-mm-ss").format(LocalDateTime.now());
            String path = "/U/"+time+".wpilog";
            Logger.addDataReceiver(new WPILOGWriter(path)); // Log to a USB stick
            Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
            LoggedPowerDistribution.getInstance(1, ModuleType.kRev); // Enables power distribution logging
            Logger.start();
        }
    }

    public static void logToSmartDashboard(String key, double value) {
        SmartDashboard.putNumber(key, value);
    }

    public static void logToSmartDashboard(String key, String value) {
        SmartDashboard.putString(key, value);
    }

    public static void logToSmartDashboard(String key, boolean value) {
        SmartDashboard.putBoolean(key, value);
    }

    public static void logSendable(Sendable sendable) {
        SmartDashboard.putData(sendable);
    }
}
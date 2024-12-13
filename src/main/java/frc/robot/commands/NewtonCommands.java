package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.PivotSubsystem.Positions;
import frc.robot.subsystems.SwerveSubsystem.DriveModes;

public final class NewtonCommands {
    private static SubsystemManager manager;


    public static void initialize(SubsystemManager manager) {
        NewtonCommands.manager = manager;
    }

    /**
     * Command to drive the swerve with translation processed for human input and
     * rotation controlled by the snap-to PID controller (snapping to the passed-in)
     * angle
     *
     * @param angle the angle to snap to
     * @param driveX a lambda returning the driver's X input
     * @param driveY a lambda returning the driver's Y input
     *
     * @return the command
     */
    public static Command swerveSnapToCommand(Rotation2d angle, DoubleSupplier driveX, DoubleSupplier driveY){
        return manager.getSwerve().run(() -> {
            ChassisSpeeds processed = manager.getSwerve().processJoystickInputs(
                driveX.getAsDouble(),
                driveY.getAsDouble(),
                0
            );
            processed.omegaRadiansPerSecond = manager.getSwerve().snapToAngle(angle);
            manager.getSwerve().drive(processed, DriveModes.AUTOMATIC);
        });
    }
    /**
     * Sets the desired position of the pivot motor depending on set locations
     *
     * @param position
     * @return the command
     */
    public static Command setPivotPositionCommand(Positions position){
        return manager.getPivot().run(() -> {
            manager.getPivot().setPosition(position);
        });
    }

    /**
     * Command to stop the intake and stow the pivot to REST position
     * @return the command
     */
    public static Command stowCommand(){
        return setPivotPositionCommand(Positions.REST).alongWith(
            manager.getIntake().getStopCommand()
        );
    }

    /**
     * Sets the intake motors to the intaking velocity in RPM
     * @return the command
     */
    public static Command setRollerIntaking() {
        return runIntakeCommand(
            Constants.INTAKE.TOP_INTAKING_RPM,
            Constants.INTAKE.BOTTOM_INTAKING_RPM
        );
    }

    /**
     * Sets the intake motors to the intaking velocity in RPM
     * @return the command
     */
    public static Command setRollerOuttaking() {
        return runIntakeCommand(
            Constants.INTAKE.TOP_OUTTAKING_RPM,
            Constants.INTAKE.BOTTOM_OUTTAKING_RPM
        );
    }

    /**
     * Sets the intake motors to the intaking velocity in RPM
     * @return the command
     */
    public static Command setRollerScoring() {
        return runIntakeCommand(
            Constants.INTAKE.TOP_SCORING_RPM,
            Constants.INTAKE.BOTTOM_SCORING_RPM
        );
    }

    /**
     * Sets the intake motors to the desired velocity in RPM
     * @return the command
     */
    public static Command runIntakeCommand(double desiredTopRPM, double desiredBottomRPM){
        return manager.getIntake().run(() -> {
            manager.getIntake().setVelocity(desiredTopRPM, desiredBottomRPM);
        });
    }
}
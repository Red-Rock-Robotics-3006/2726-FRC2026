package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tank;
import redrocklib.wrappers.logging.SmartDashboardNumber;

public class Autos {

    private static Shooter shooter = Shooter.getInstance();
    private static Intake intake = Intake.getInstance();
    private static Tank tank = Tank.getInstance();

    private static SmartDashboardNumber shootSeconds = new SmartDashboardNumber("autos/shooter-seconds", 8);
    private static SmartDashboardNumber autoDriveSpeed = new SmartDashboardNumber("Autos/auto-drive-speed", 0.3);
    private static SmartDashboardNumber autoTurnSpeed = new SmartDashboardNumber("Autos/auto-turn-speed", 0.3);
    private static SmartDashboardNumber turnSeconds = new SmartDashboardNumber("Autos/auto-turn-seconds", 1);
    private static SmartDashboardNumber driveSeconds = new SmartDashboardNumber("Autos/auto-turn-seconds", 3);
    private static SmartDashboardNumber preloadSeconds = new SmartDashboardNumber("Autos/preload-wiat-seconds", 5);

    public static Command awayHubPreload(){
        return Commands.sequence(
            shooter.shootCommandAwayHub(),
            Commands.waitSeconds(shootSeconds.getNumber()),
            shooter.stopShooterCommand()
        );
    }

    public static Command hubPreload(){
        return Commands.sequence(
            shooter.shootCommandHub(),
            Commands.waitSeconds(shootSeconds.getNumber()),
            shooter.stopShooterCommand()
        );
    }

    public static Command goToMiddle(){
        return Commands.sequence(
            shooter.autoAimShootCommand(),
            Commands.waitSeconds(preloadSeconds.getNumber()),
            shooter.stopShooterCommand(),
            intake.resetIntakeCommand(),
            intake.runIntakeCommand(),
            Commands.runOnce(() -> tank.drive(autoTurnSpeed.getNumber(), 0), tank),
            Commands.waitSeconds(turnSeconds.getNumber()),
            Commands.runOnce(() -> tank.drive(0, autoDriveSpeed.getNumber()), tank),
            Commands.waitSeconds(driveSeconds.getNumber()),
            Commands.runOnce(() -> tank.drive(0,0), tank)
        );
    }
}
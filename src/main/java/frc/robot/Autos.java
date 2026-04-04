package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Shooter;
import redrocklib.wrappers.logging.SmartDashboardNumber;

public class Autos {

    private static Shooter shooter = Shooter.getInstance();

    private static SmartDashboardNumber shootSeconds = new SmartDashboardNumber("autos/shooter-seconds", 8);

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
}
package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Shooter;
import redrocklib.logging.SmartDashboardNumber;

public class Autos {

    private static Shooter shooter = Shooter.getInstance();

    private static SmartDashboardNumber shootSeconds = new SmartDashboardNumber("autos/shooter-seconds", 5);

    public static Command midPreload(){
        return Commands.sequence(
            shooter.autoAimShootCommand(),
            Commands.waitSeconds(shootSeconds.getNumber()),
            shooter.stopShooterCommand()
        );
    }

    public static Command rightPreload(){
        return Commands.sequence(
            shooter.autoAimShootCommand(),
            Commands.waitSeconds(shootSeconds.getNumber()),
            shooter.stopShooterCommand()
        );
    }

    public static Command leftPreload(){
        return Commands.sequence(
            shooter.autoAimShootCommand(),
            Commands.waitSeconds(shootSeconds.getNumber()),
            shooter.stopShooterCommand()
        );
    }
}
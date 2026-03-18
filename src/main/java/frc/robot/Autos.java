package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class Autos {

    private static Shooter shooter = Shooter.getInstance();

    public static Command midPreload(){
        return shooter.autoAimShootCommand();
    }

    public static Command rightPreload(){
        return shooter.autoAimShootCommand();
    }

    public static Command leftPreload(){
        return shooter.autoAimShootCommand();
    }
}
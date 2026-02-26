package frc.robot;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tank;

public class Autos {
    private final AutoFactory factory;

    private Intake intake = Intake.getInstance();
    private Conveyor conveyor = Conveyor.getInstance();
    private Tank driveTrain = Tank.getInstance();
    private Shooter shooter = Shooter.getInstance();
    private LED led = LED.getInstance();

    public Autos(AutoFactory f){
        this.factory = f;
    }

    public Command midPile(){
        return Commands.sequence(
            this.shooter.shootIndexCommand(),
            this.factory.trajectoryCmd("MidPile", 0),
            this.intake.startIntakingCommand(),
            this.factory.trajectoryCmd("MidPile", 1),
            this.intake.stopIntakingCommand(),
            this.shooter.shootIndexCommand()
        );
    }
    
    public Command rightPile(){
        return Commands.sequence(
            this.shooter.shootIndexCommand(),
            this.factory.trajectoryCmd("RightPile", 0),
            this.intake.startIntakingCommand(),
            this.factory.trajectoryCmd("RightPile", 1),
            this.intake.stopIntakingCommand(),
            this.shooter.shootIndexCommand()
        );
    }

    public Command midPreload(){
        return this.shooter.shootIndexCommand();
    }

    public Command rightPreload(){
        return this.shooter.shootIndexCommand();
    }

    public Command leftPreload(){
        return this.shooter.shootIndexCommand();
    }
}

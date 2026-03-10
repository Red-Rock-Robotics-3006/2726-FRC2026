package frc.robot;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tank;

public class Autos {
    private final AutoFactory factory;

    private Intake intake = Intake.getInstance();
    private Tank tank = Tank.getInstance();
    private Shooter shooter = Shooter.getInstance();
    private LED led = LED.getInstance();

    public Autos(AutoFactory f){ //TODO tank driveRequest
        this.factory = f;
    }

    public Command midPile(){
        return Commands.sequence(
            this.factory.trajectoryCmd("MidPile", 0),
            this.intake.deployIntakeCommand(),
            this.factory.trajectoryCmd("MidPile", 1),
            this.factory.trajectoryCmd("MidPile", 2),
            this.factory.trajectoryCmd("MidPile", 3),
            this.intake.stowIntakeCommand(),
            this.factory.trajectoryCmd("MidPile", 4),
            this.shooter.autoAimshootCommand()
        );
    }

    public Command leftPile(){
        return Commands.sequence(
            this.shooter.autoAimshootCommand(),
            this.factory.trajectoryCmd("LeftPile", 0),
            this.factory.trajectoryCmd("LeftPile", 1),
            this.intake.deployIntakeCommand(),
            this.factory.trajectoryCmd("MidPile", 2),
            this.factory.trajectoryCmd("MidPile", 3),
            this.intake.stowIntakeCommand(),
            this.factory.trajectoryCmd("MidPile", 4),
            this.shooter.autoAimshootCommand()
        );
    }
    
    public Command rightPile(){
        return Commands.sequence(
            this.shooter.autoAimshootCommand(),
            this.factory.trajectoryCmd("RightPile", 0),
            this.factory.trajectoryCmd("RightPile", 1),
            this.intake.deployIntakeCommand(),
            this.factory.trajectoryCmd("RightPile", 2),
            this.factory.trajectoryCmd("RightPile", 3),
            this.intake.stowIntakeCommand(),
            this.factory.trajectoryCmd("RightPile", 4),
            this.factory.trajectoryCmd("RightPile", 5),
            this.shooter.autoAimshootCommand()
        );
    }

    public Command rightHuman(){
        return Commands.sequence(
            this.shooter.autoAimshootCommand(),
            this.factory.trajectoryCmd("RightHuman", 0),
            Commands.waitSeconds(2),
            this.factory.trajectoryCmd("RightHuman", 1),
            this.shooter.autoAimshootCommand()
        );
    }

    public Command midPreload(){
        return this.shooter.autoAimshootCommand();
    }

    public Command rightPreload(){
        return this.shooter.autoAimshootCommand();
    }

    public Command leftPreload(){
        return this.shooter.autoAimshootCommand();
    }
}

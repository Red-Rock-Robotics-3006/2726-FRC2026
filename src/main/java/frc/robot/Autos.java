package frc.robot;

import choreo.auto.AutoFactory;
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


}

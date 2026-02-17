package frc.robot;

import choreo.auto.AutoFactory;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Tank;

public class Autos {
    private final AutoFactory factory;

    private Intake intake = Intake.getInstance();
    private Conveyor conveyor = Conveyor.getInstace();
    private Tank driveTrain = Tank.getInstance();

    public Autos(AutoFactory f){
        this.factory = f;
    }


}

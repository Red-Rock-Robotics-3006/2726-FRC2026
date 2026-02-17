package frc.robot.subsystems;


import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
// import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import redrocklib.logging.SmartDashboardNumber;

public class Conveyor extends SubsystemBase{
    private SparkMax conveyorMotor = new SparkMax(18, MotorType.kBrushless);


    private SmartDashboardNumber conveyorSpeed = new SmartDashboardNumber("Conveyor/conveyorSpeedPercent(uses motor.set)", 0);
    
    
    private static Conveyor instance = null;
    
    public Conveyor(){
        super();

        SparkMaxConfig conveyorConfig = new SparkMaxConfig();
        conveyorConfig.idleMode(IdleMode.kBrake).inverted(false);

        conveyorMotor.configure(conveyorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setConveyorSpeed(double speed){
        conveyorMotor.set(speed);
    }

    public void conveyToShooter(){
        this.setConveyorSpeed(this.conveyorSpeed.getNumber());
    }

    public void conveyToShooterPassive(){
        this.setConveyorSpeed(this.conveyorSpeed.getNumber()*0.3);
    }

    //Will be pretty useless considering the hopper is angled down to the shooter, most likely will just spin the balls
    public void conveyBack(){
        this.setConveyorSpeed(-this.conveyorSpeed.getNumber());
    }

    public Command spinActiveCommand(){
        return Commands.runOnce(() -> this.conveyToShooter(),this);
    }
    public Command spinPassiveCommand(){
        return Commands.runOnce(() -> this.conveyToShooterPassive(),this);
    }
    public Command reverseConveyorCommand(){
        return Commands.runOnce(() -> this.conveyBack(),this);
    }

    public Command stopConveyorCommand(){
        return Commands.runOnce(() -> this.setConveyorSpeed(0),this);
    }

    public static Conveyor getInstance(){
        if (instance == null) {
            instance = new Conveyor();
        }
        return instance;
    }

    @Override
    public void periodic(){
        if(this.conveyorSpeed.hasChanged()) this.conveyorSpeed.putNumber(this.conveyorSpeed.getNumber());
    }

}

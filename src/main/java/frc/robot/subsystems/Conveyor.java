package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import redrocklib.logging.SmartDashboardNumber;

public class Conveyor extends SubsystemBase {
    private static Conveyor instance = null;

    private final SparkMax conveyMotor = new SparkMax(3, MotorType.kBrushless); //TODO

    private final SparkClosedLoopController controller = conveyMotor.getClosedLoopController();

    private SparkMaxConfig config = new SparkMaxConfig();

    private SmartDashboardNumber feedForwardSpeed = new SmartDashboardNumber("Conveyor/feedForwardSpeed", 0);  //TODO
    private SmartDashboardNumber feedBackSpeed = new SmartDashboardNumber("Conveyor/feedBackSpeed", 0);  //TODO
    private SmartDashboardNumber kP = new SmartDashboardNumber("Conveyor/kP", 0);  //TODO
    private SmartDashboardNumber kI = new SmartDashboardNumber("Conveyor/kI", 0);  //TODO
    private SmartDashboardNumber kD = new SmartDashboardNumber("Conveyor/kD", 0);  //TODO

    private Conveyor(){
        super("Conveyor");

        config.idleMode(IdleMode.kBrake);
        config.closedLoop
            .p(kP.getNumber())
            .i(kI.getNumber())
            .d(kD.getNumber());
        conveyMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setFeedForwardSpeed(){
        this.controller.setSetpoint(feedForwardSpeed.getNumber(), ControlType.kVelocity);
    }

    public void setFeedBackSpeed(){
        this.controller.setSetpoint(feedBackSpeed.getNumber(), ControlType.kVelocity);
    }

    public void stop(){
        this.conveyMotor.set(0);
    }

    public Command feedConveyorForwardCommand(){
        return Commands.runOnce(() -> this.setFeedForwardSpeed());
    }

    public Command feedConveyorBackwardCommand(){
        return Commands.runOnce(() -> this.setFeedBackSpeed());
    }

    public Command stopConveyorCommand(){
        return Commands.runOnce(() -> this.stop());
    }

    @Override
    public void periodic(){
        if(kP.hasChanged()
        || kI.hasChanged()
        || kD.hasChanged()){
            config.closedLoop
                .p(kP.getNumber())
                .i(kI.getNumber())
                .d(kD.getNumber());
        }
        SmartDashboard.putNumber("Conveyor/conveyor-current-position", conveyMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Conveyor/conveyor-current-velocity", conveyMotor.getEncoder().getVelocity());
    }

    public static Conveyor getInstance(){
        if(instance == null) instance = new Conveyor();
        return instance;
    }
}

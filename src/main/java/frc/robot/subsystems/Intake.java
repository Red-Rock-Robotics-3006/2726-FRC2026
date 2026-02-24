package frc.robot.subsystems;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import redrocklib.logging.SmartDashboardNumber;


public class Intake extends SubsystemBase{
    private static Intake instance = null;

    private final SparkMax intakeMotor = new SparkMax(0, MotorType.kBrushless); //TODO
    private final SparkMax hingeMotor = new SparkMax(0, MotorType.kBrushless); //TODO

    private final SparkClosedLoopController intake_controller = intakeMotor.getClosedLoopController();
    private final SparkClosedLoopController hinge_controller = hingeMotor.getClosedLoopController();
 
    private SparkMaxConfig intakeConfig = new SparkMaxConfig();
    private SparkMaxConfig hingeConfig = new SparkMaxConfig();

    private SmartDashboardNumber intakeSpeed = new SmartDashboardNumber("Intake/intakeSpeed", 0.3); //TODO
    private SmartDashboardNumber outtakeSpeed = new SmartDashboardNumber("Intake/outtakeSpeed", -0.3); //TODO
    private SmartDashboardNumber stowPosition = new SmartDashboardNumber("Intake/stowPosition", 0); //TODO
    private SmartDashboardNumber deployPosition = new SmartDashboardNumber("Intake/deployPosition", 0); //TODO
    private SmartDashboardNumber intakekP = new SmartDashboardNumber("Intake/intakekP", 0); //TODO
    private SmartDashboardNumber intakekI = new SmartDashboardNumber("Intake/intakekI", 0); //TODO
    private SmartDashboardNumber intakekD = new SmartDashboardNumber("Intake/intakekD", 0); //TODO
    private SmartDashboardNumber hingekP = new SmartDashboardNumber("Intake/hingekP", 0); //TODO    

    
    private SmartDashboardNumber hingekI = new SmartDashboardNumber("Intake/hingekI", 0); //TODO
    private SmartDashboardNumber hingekD = new SmartDashboardNumber("Intake/hingekD", 0); //TODO
    
    private Intake(){

        super("Intake");
        
        intakeConfig.idleMode(IdleMode.kBrake);
        intakeConfig.closedLoop  //TODO
            .p(intakekP.getNumber())
            .i(intakekI.getNumber())
            .d(intakekD.getNumber())
            .outputRange(-0.8, 0.8);

        intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        
        
        hingeConfig.idleMode(IdleMode.kBrake);
        hingeConfig.closedLoop  //TODO
            .p(hingekP.getNumber())
            .i(hingekI.getNumber())
            .d(hingekD.getNumber())
            .outputRange(-0.8, 0.8);

        hingeMotor.configure(hingeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setOttakeSpeedRPM(){
        //this.intake_controller.setSetpoint(this.outtakeSpeed.getNumber(), ControlType.kVelocity);
    }

    public void setIntakeSpeedRPM(){
        this.intake_controller.setSetpoint(this.intakeSpeed.getNumber(), ControlType.kVelocity);
    }

    public void stopIntake(){
        intakeMotor.set(0);
    }

    public void setIntakeStowPosition(){
        this.hinge_controller.setSetpoint(this.stowPosition.getNumber(), ControlType.kPosition);
    }

    public void setIntakeDeployPosition(){
        this.hinge_controller.setSetpoint(this.deployPosition.getNumber(), ControlType.kPosition);
    }

    public boolean isIntaking(){
        return hingeMotor.getEncoder().getPosition() > 5;
    }

    // public void normalizeIntake(){
    //     this.hinge_controller.set(-0.1);
    // }

    // public void resetIntake(){
    //     this.hingeMotor.motor.setControl(new NeutralOut());
    // }

    public Command startIntakingCommand(){
        return Commands.sequence(
            Commands.runOnce(() -> this.setIntakeDeployPosition()),
            Commands.runOnce(() -> this.setIntakeSpeedRPM())
        );
    }

    public Command startOuttakingCommand(){
        return Commands.sequence(
            Commands.runOnce(() -> this.setIntakeDeployPosition()),
            Commands.runOnce(() -> this.setOttakeSpeedRPM())
        );
    }

    public Command stopIntakingCommand(){
        return Commands.sequence(
            Commands.runOnce(() -> this.setIntakeStowPosition()),
            Commands.runOnce(() -> this.stopIntake())
        );
    }

    @Override
    public void periodic(){
        if(intakekP.hasChanged() 
        || intakekI.hasChanged()
        || intakekD.hasChanged()
        || hingekP.hasChanged()
        || hingekI.hasChanged()
        || hingekD.hasChanged()){
            intakeConfig.closedLoop
                .p(intakekP.getNumber())
                .i(intakekI.getNumber())
                .d(intakekD.getNumber());
            hingeConfig.closedLoop
                .p(hingekP.getNumber())
                .i(hingekI.getNumber())
                .d(hingekD.getNumber());
        }
        SmartDashboard.putNumber("intake/intake-current-position", intakeMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("intake/intake-current-velocity", intakeMotor.getEncoder().getVelocity());
        // SmartDashboard.putNumber("intake/intake-current-acceleration", intakeMotor.getEncoder().getAcceleration());
        // SmartDashboard.putNumber("intake/intake-torque-current",intakeMotor.getEncoder().getTorqueCurrent());

    }

    public static Intake getInstance(){
        if(instance == null) instance = new Intake();
        return instance;
    }
}

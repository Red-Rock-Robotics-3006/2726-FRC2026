package frc.robot.subsystems;

import com.ctre.phoenix6.controls.NeutralOut;
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

public class Intake extends SubsystemBase{
    public static Intake instance;
    SparkMax hingeMotor = new SparkMax(49, MotorType.kBrushless);
    SparkMax rollerMotor = new SparkMax(49, MotorType.kBrushless);

    SparkClosedLoopController hingeMotorController = hingeMotor.getClosedLoopController();
    SparkClosedLoopController rollerMotorController = rollerMotor.getClosedLoopController();
    
    SmartDashboardNumber kPHinge = new SmartDashboardNumber( "Intake/kPHinge",0);
    SmartDashboardNumber kIHinge = new SmartDashboardNumber( "Intake/kIHinge",0);
    SmartDashboardNumber kDHinge = new SmartDashboardNumber( "Intake/kDHinge",0);
    SmartDashboardNumber kMinOutputHinge = new SmartDashboardNumber("Intake/kMinOutputHinge", 0);
    SmartDashboardNumber kMaxOutputHinge = new SmartDashboardNumber("Intake/kMaxOutputHinge", 0);

    SmartDashboardNumber kPRoller = new SmartDashboardNumber( "Intake/kPRoller",0);
    SmartDashboardNumber kIRoller = new SmartDashboardNumber( "Intake/kIRoller",0);
    SmartDashboardNumber kDRoller = new SmartDashboardNumber( "Intake/kDRoller",0);
    SmartDashboardNumber kMinOutputRoller = new SmartDashboardNumber("Intake/kMinOutputRoller", 0);
    SmartDashboardNumber kMaxOutputRoller = new SmartDashboardNumber("Intake/kMaxOutputRoller", 0);
    
    SmartDashboardNumber intakeSpeed = new SmartDashboardNumber("Intake/intakeSpeed",0);
    SmartDashboardNumber backSpeed = new SmartDashboardNumber("Intake/backSpeed",0);
    SmartDashboardNumber deployPos = new SmartDashboardNumber("Intake/deployPos", 0);
    SmartDashboardNumber stowPos = new SmartDashboardNumber("Intake/stowPos", 0);

    public Intake(){
        super();
        SparkMaxConfig hingeConfig = new SparkMaxConfig();
        hingeConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(50);
        hingeConfig.closedLoop
            .p(kPHinge.getNumber())
            .i(kIHinge.getNumber())
            .d(kDHinge.getNumber())
            .outputRange(kMinOutputHinge.getNumber(), kMaxOutputHinge.getNumber());

        SparkMaxConfig rollerConfig = new SparkMaxConfig();
        rollerConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(50);
        rollerConfig.closedLoop
            .p(kPRoller.getNumber())
            .i(kIRoller.getNumber())
            .d(kDRoller.getNumber())
            .outputRange(kMinOutputRoller.getNumber(), kMaxOutputRoller.getNumber());
    }
    
    public void deployIntake(){
        this.hingeMotorController.setSetpoint(deployPos.getNumber(), ControlType.kPosition);
    }

    public void stowIntake(){
        this.hingeMotorController.setSetpoint(stowPos.getNumber(), ControlType.kPosition);
    }

    public void setIntakeSpeed(double speed){
        this.rollerMotorController.setSetpoint(speed,ControlType.kVelocity);
    }
    public void startIntaking(){
        this.setIntakeSpeed(intakeSpeed.getNumber());
    }

    public void regurgitateIntake(){
        this.setIntakeSpeed(backSpeed.getNumber());;
    }

    public void stopIntakeRoller(){
        this.setIntakeSpeed(0);
    }

    public void normalizeIntake(){
        this.rollerMotor.set(-0.1);
    }
// do sumthin abt this idk
    public void resetIntake(){
        this.stopIntakeRoller();
        // this.hingeMotor.
        
    }

    public Command spinRollerCommand(){
        return Commands.runOnce(() -> this.startIntaking(), this);
    }

    public Command regurgitIntakeCommand(){
        return Commands.runOnce(() -> this.regurgitateIntake(), this);
    }

    public Command stopIntakeRollerCommand(){
        return Commands.runOnce(() -> this.stopIntakeRoller(), this);
    }

    public Command deployCommand(){
        return Commands.sequence(
            Commands.runOnce(() -> this.deployIntake(), this),
            this.spinRollerCommand()
        );
    }

    public Command regurgitateCommand(){
        return Commands.sequence(
            Commands.runOnce(() -> this.deployIntake(), this),
            this.regurgitIntakeCommand()
        );
    }

    public Command stowCommand(){
        return Commands.sequence(
            Commands.runOnce(() -> this.stowIntake(), this),
            this.stopIntakeRollerCommand()
        );
    }

    public static Intake getInstance(){
        if(Intake.instance == null){
            Intake.instance = new Intake();
        }
        return Intake.instance;
    }
    @Override
    public void periodic(){
        if(this.kDHinge.hasChanged()
        ||this.kPHinge.hasChanged() 
        ||this.kIHinge.hasChanged() 
        ||this.kMinOutputHinge.hasChanged() 
        ||this.kPRoller.hasChanged() 
        ||this.kIRoller.hasChanged() 
        ||this.kDRoller.hasChanged() 
        ||this.kMinOutputRoller.hasChanged() 
        ||this.kMaxOutputRoller.hasChanged() 
        ){
            this.kPHinge = SmartDashboard.getNumber("Intake/kPhinge",kPHinge);
        }
    }

}

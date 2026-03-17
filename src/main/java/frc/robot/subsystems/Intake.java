package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;

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
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import redrocklib.logging.SmartDashboardNumber;

public class Intake extends SubsystemBase{
    public static Intake instance = null;
    private final SparkMax hingeMotor = new SparkMax(49, MotorType.kBrushless);
    private final SparkMax rollerMotor = new SparkMax(49, MotorType.kBrushless);

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

    SparkMaxConfig hingeConfig = new SparkMaxConfig();
    SparkMaxConfig rollerConfig = new SparkMaxConfig();

    private double currentThreshold = 50;

    public Intake(){
        super();
        AutoLogOutputManager.addObject(this);
        hingeConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(60);
        hingeConfig.closedLoop
            .p(kPHinge.getNumber())
            .i(kIHinge.getNumber())
            .d(kDHinge.getNumber())
            .outputRange(kMinOutputHinge.getNumber(), kMaxOutputHinge.getNumber());

        rollerConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(60);
        rollerConfig.closedLoop
            .p(kPRoller.getNumber())
            .i(kIRoller.getNumber())
            .d(kDRoller.getNumber())
            .outputRange(kMinOutputRoller.getNumber(), kMaxOutputRoller.getNumber());
    }
    
    private void deployIntake(){
        this.hingeMotorController.setSetpoint(deployPos.getNumber(), ControlType.kPosition);
        Logger.recordOutput("Intake/Position", "DEPLOYED");
    }
    
    private void stowIntake(){
        Logger.recordOutput("Intake/Position", "STOWED");
        this.hingeMotorController.setSetpoint(stowPos.getNumber(), ControlType.kPosition);
    }

    private void setIntakeSpeedRPM(double speed){
        this.rollerMotorController.setSetpoint(speed,ControlType.kVelocity);
    }
    private void startIntaking(){
        this.setIntakeSpeedRPM(intakeSpeed.getNumber());
        Logger.recordOutput("Intake/Status", "INTAKING");
    }

    private void regurgitateIntake(){
        this.setIntakeSpeedRPM(backSpeed.getNumber());
        Logger.recordOutput("Intake/Status", "REVERSE INTAKE DIRECTION");
    }

    private void stopIntakeRoller(){
        this.setIntakeSpeedRPM(0);
    }

    private void normalizeIntake(){
        this.rollerMotor.set(-0.1);
    }
// do sumthin abt this idk
    private void resetIntake(){
        this.hingeMotor.getEncoder().setPosition(0);
    }

    public boolean isIntaking(){
        if(hingeMotor.getEncoder().getPosition() > 5) 
            Logger.recordOutput("Intake/Position", "INTAKING");
        return hingeMotor.getEncoder().getPosition() > 5; //TODO
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

    public Command deployIntakeCommand(){
        return Commands.sequence(
            Commands.runOnce(() -> this.deployIntake(), this),
            this.spinRollerCommand()
        );
    }

    public Command regurgitateIntakeCommand(){
        return Commands.sequence(
            Commands.runOnce(() -> this.deployIntake(), this),
            this.regurgitIntakeCommand()
        );
    }

    public Command stowIntakeCommand(){
        return Commands.sequence(
            Commands.runOnce(() -> this.stowIntake(), this),
            this.stopIntakeRollerCommand()
        );
    }

    public Command resetIntakeCommand(){
        return new FunctionalCommand(
            () -> this.normalizeIntake(),
            () -> {},
            (interrupted) -> this.resetIntake(),
            () -> {
                return this.hingeMotor.getOutputCurrent() > currentThreshold;
            }, 
            this
        );
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
        this.hingeConfig.closedLoop.p(this.kPHinge.getNumber()).i(this.kIHinge.getNumber()).d(this.kDHinge.getNumber()).minOutput(this.kMinOutputHinge.getNumber());
        this.rollerConfig.closedLoop.p(this.kPRoller.getNumber()).i(this.kIRoller.getNumber()).d(this.kDRoller.getNumber()).minOutput(this.kMinOutputRoller.getNumber()).maxOutput(this.kMaxOutputHinge.getNumber());
        SmartDashboard.putNumber("intake/intake-current-position", hingeMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("intake/intake-current-velocity", hingeMotor.getEncoder().getVelocity());
        }
    }

    public static Intake getInstance(){
        if(Intake.instance == null){
            Intake.instance = new Intake();
        }
        return Intake.instance;
    }

}

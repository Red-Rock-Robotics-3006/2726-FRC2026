package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.NeutralOut;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import redrocklib.logging.SmartDashboardNumber;

public class Intake extends SubsystemBase{
    public static Intake instance = null;
    private final SparkFlex hingeMotor = new SparkFlex(22, MotorType.kBrushless);
    private final SparkFlex rollerMotor = new SparkFlex(21, MotorType.kBrushless);

    SparkClosedLoopController hingeMotorController = hingeMotor.getClosedLoopController();
    SparkClosedLoopController rollerMotorController = rollerMotor.getClosedLoopController();
    
    SmartDashboardNumber kPHinge = new SmartDashboardNumber( "Intake/kPHinge",0.08);
    SmartDashboardNumber kIHinge = new SmartDashboardNumber( "Intake/kIHinge",0.0);
    SmartDashboardNumber kDHinge = new SmartDashboardNumber( "Intake/kDHinge",0.02);
    SmartDashboardNumber maxVelocitytHinge = new SmartDashboardNumber("Intake/MaxVelocityHinge", .00020);
    SmartDashboardNumber maxAccel = new SmartDashboardNumber("Intake/MaxAccel", .20);

    SmartDashboardNumber kPRoller = new SmartDashboardNumber( "Intake/kPRoller",0.2);
    SmartDashboardNumber kIRoller = new SmartDashboardNumber( "Intake/kIRoller",0);
    SmartDashboardNumber kDRoller = new SmartDashboardNumber( "Intake/kDRoller",0);
    // SmartDashboardNumber kMinOutputRoller = new SmartDashboardNumber("Intake/kMinOutputRoller", -1);
    // SmartDashboardNumber kMaxOutputRoller = new SmartDashboardNumber("Intake/kMaxOutputRoller", 1);
    
    SmartDashboardNumber intakeSpeed = new SmartDashboardNumber("Intake/intakeSpeed",-0.1);
    SmartDashboardNumber rollerSpeedBack = new SmartDashboardNumber("Intake/backSpeed",0.1);
    SmartDashboardNumber deployPos = new SmartDashboardNumber("Intake/deployPos", -10.433);
    SmartDashboardNumber stowPos = new SmartDashboardNumber("Intake/stowPos", 0);

    SmartDashboardNumber intakePos = new SmartDashboardNumber("Intake/intake-position", 0);
    SmartDashboardNumber rollerSpeed = new SmartDashboardNumber("Intake/roller-speed", 0.6);

    SparkFlexConfig hingeConfig = new SparkFlexConfig();
    SparkFlexConfig rollerConfig = new SparkFlexConfig();

    private SmartDashboardNumber currentThreshold = new SmartDashboardNumber("Intake/current-threshold", 25);

    public Intake(){
        super();
        AutoLogOutputManager.addObject(this);
        hingeConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(60);
        hingeConfig.closedLoop
            .p(kPHinge.getNumber())
            .i(kIHinge.getNumber())
            .d(kDHinge.getNumber())
            .maxMotion.cruiseVelocity(maxVelocitytHinge.getNumber())
            .maxAcceleration(maxAccel.getNumber());
            // .outputRange(kMinOutputHinge.getNumber(), kMaxOutputHinge.getNumber());

        rollerConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(60);
        rollerConfig.closedLoop
            .p(kPRoller.getNumber())
            .i(kIRoller.getNumber())
            .d(kDRoller.getNumber());
            // .outputRange(kMinOutputRoller.getNumber(), kMaxOutputRoller.getNumber());

        this.hingeMotor.configure(hingeConfig, com.revrobotics.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        this.rollerMotor.configure(rollerConfig, com.revrobotics.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    
    
    private void deployIntake(){
        this.hingeMotorController.setSetpoint(deployPos.getNumber(), ControlType.kPosition);
        Logger.recordOutput("Intake/Position", "DEPLOYED");
    }
    
    private void stowIntake(){
        Logger.recordOutput("Intake/Position", "STOWED");
        this.hingeMotorController.setSetpoint(stowPos.getNumber(), ControlType.kPosition);
        // this.hingeMotorController.getMAXMotionSetpointPosition()
    }

    private void setIntakeSpeed(double speed){
        this.rollerMotor.set(speed);
    }
    private void startIntaking(){
        this.setIntakeSpeed(-rollerSpeed.getNumber());
        Logger.recordOutput("Intake/Status", "INTAKING");
    }

    private void stopIntake(){
        this.hingeMotor.set(0);
    }

    private void regurgitateIntake(){
        this.setIntakeSpeed(rollerSpeedBack.getNumber());
        Logger.recordOutput("Intake/Status", "REVERSE INTAKE DIRECTION");
    }

    private void stopIntakeRoller(){
        this.setIntakeSpeed(0);
    }

    private void normalizeIntake(){
        this.hingeMotor.set(0.05);
    }
// do sumthin abt this idk
    private void resetIntake(){
        this.stopIntake();
        this.hingeMotor.getEncoder().setPosition(0.4);
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
                return this.hingeMotor.getOutputCurrent() > currentThreshold.getNumber();
            }, 
            this
        );
    }

    @Override
    public void periodic(){
        if(this.kDHinge.hasChanged()
        ||this.kPHinge.hasChanged() 
        ||this.kIHinge.hasChanged() 
        // ||this.kMinOutputHinge.hasChanged() 
        ||this.kPRoller.hasChanged() 
        ||this.kIRoller.hasChanged() 
        ||this.kDRoller.hasChanged() 
        // ||this.kMinOutputRoller.hasChanged() 
        // ||this.kMaxOutputRoller.hasChanged() 
        ||this.maxVelocitytHinge.hasChanged()
        ){
            this.hingeConfig.closedLoop.p(this.kPHinge.getNumber()).i(this.kIHinge.getNumber()).d(this.kDHinge.getNumber()).maxMotion.cruiseVelocity(maxVelocitytHinge.getNumber()).maxAcceleration(maxAccel.getNumber());
            this.rollerConfig.closedLoop.p(this.kPRoller.getNumber()).i(this.kIRoller.getNumber()).d(this.kDRoller.getNumber());
        }
    
        SmartDashboard.putNumber("Intake/Requested-set-value", hingeMotor.getAppliedOutput());
        SmartDashboard.putNumber("Intake/intake-current-velocity", hingeMotor.getEncoder().getVelocity());
        this.intakePos.putNumber(this.hingeMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Intake/current", this.hingeMotor.getOutputCurrent());
        SmartDashboard.putNumber("Intake/current-rollers", this.rollerMotor.getOutputCurrent());
    }

    public static Intake getInstance(){
        if(Intake.instance == null){
            Intake.instance = new Intake();
        }
        return Intake.instance;
    }

}

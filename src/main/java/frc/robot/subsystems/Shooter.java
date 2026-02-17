package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
// import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
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
import redrocklib.wrappers.RedRockTalon;


 
public class Shooter extends SubsystemBase{
    public static Shooter instance = null;

    private Tank tank = Tank.getInstance();
    private LED leds = LED.getInstance();
    private RedRockTalon shooterMotor = new RedRockTalon(45, "shooterMotor");
    private SparkMax indexMotor = new SparkMax(45,MotorType.kBrushless);
    private boolean tankFrozen = false;

    SparkMaxConfig indexConfig = new SparkMaxConfig();

    SparkClosedLoopController indexController = indexMotor.getClosedLoopController();
    private SmartDashboardNumber shootSpeed = new SmartDashboardNumber("Shooter/shootingSpeedRPM", 0);
    private SmartDashboardNumber shootSpeedReverse = new SmartDashboardNumber("Shooter/shootingSpeedReverseRPM", 0);
    private SmartDashboardNumber indexSpeed = new SmartDashboardNumber("Shooter/indexShootSpeedRPM", 0);
    private SmartDashboardNumber indexDeclogSpeed = new SmartDashboardNumber("Shooter/indexDeclogSpeedRPM", 0);
    private SmartDashboardNumber shooterTriggerVelocity = new SmartDashboardNumber("Shooter/shooterTriggerVelocity", 0);

    private SmartDashboardNumber kPIndex = new SmartDashboardNumber("Shooter/kPIndex", 0);
    private SmartDashboardNumber kIIndex = new SmartDashboardNumber("Shooter/kIIndex", 0);
    private SmartDashboardNumber kDIndex = new SmartDashboardNumber("Shooter/kDIndex", 0);
    
    private SmartDashboardNumber kMinOutputIndex = new SmartDashboardNumber("Shooter/kMinOutputIndex", 0);
    private SmartDashboardNumber kMaxOutputIndex;

    
    public Shooter(){
        super();
        indexConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(50);
        indexConfig.closedLoop
            .p(kPIndex.getNumber())
            .i(kIIndex.getNumber())
            .d(kDIndex.getNumber())
            .outputRange(kMinOutputIndex.getNumber(), kMaxOutputIndex.getNumber());


        this.shooterMotor.withMotorOutputConfigs(
            new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withPeakForwardDutyCycle(1d)
            .withPeakReverseDutyCycle(-1d)
            .withNeutralMode(NeutralModeValue.Coast)
        )
        .withSlot0Configs(
            new Slot0Configs()
            .withKA(0)
            .withKS(0)
            .withKV(0)
            .withKP(0.05)
            .withKI(0)
        )
        .withMotionMagicConfigs(
            new MotionMagicConfigs()

            .withMotionMagicAcceleration(1300)
            .withMotionMagicCruiseVelocity(100)
        )
        .withSpikeThreshold(28)
        .withCurrentLimitConfigs(
            new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(45)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(80)
            .withStatorCurrentLimitEnable(true)
        ).withTuningEnabled(true);
    }

    public void setIndexSpeedRPM(double rpm){
        this.indexController.setSetpoint(rpm, ControlType.kVelocity);
    }

    public void setShooterSpeedRPM(double rpm){
        this.shooterMotor.setMotionMagicVelocity(rpm);
    }


//Shooter motor methods
    //WARNING : VERY BAD IDEA
    public void setShooterReverse(){
        this.setShooterSpeedRPM(this.shootSpeedReverse.getNumber());
    }

    public void startShooting(){
        this.setShooterSpeedRPM(this.shootSpeedReverse.getNumber());
    }

    public void stopShooting(){
        this.setShooterSpeedRPM(0);
    }

    public boolean isAtShootSpeed(){
        return (Math.abs(this.shooterTriggerVelocity.getNumber() - shooterMotor.motor.getVelocity().getValueAsDouble()*60.0)) <= 100;
    }
//Index motor methods
    public void indexForward(){
        this.setIndexSpeedRPM(this.indexSpeed.getNumber());
    }
    public void indexDeclog(){
        this.setIndexSpeedRPM(this.indexDeclogSpeed.getNumber());
    }
    public void stopIndexing(){
        this.setIndexSpeedRPM(0);
    }


//Commands

    public Command shootCommand(){
        return Commands.sequence(
            Commands.runOnce(() -> this.startShooting(), this),
            Commands.waitUntil(() -> this.isAtShootSpeed()),
            Commands.runOnce(() -> this.indexForward(), this)
        );
    }

    public Command stopShootingCommand(){
        return Commands.sequence(
            Commands.runOnce(() -> this.stopShooting(), this),
            Commands.runOnce(() -> this.stopIndexing(), this)
        );
    }
    

//Commands for odd situations

    public Command reverseShootCommand(){
        return Commands.sequence(
            Commands.runOnce(() -> this.setShooterReverse(), this),
            Commands.runOnce(() -> this.indexDeclog(), this)
        );
    }

//Periodic Implementation
    @Override
    public void periodic(){
        shooterMotor.update();
        if(kPIndex.hasChanged()
        || kIIndex.hasChanged()
        || kDIndex.hasChanged()
        || kMinOutputIndex.hasChanged()
        || kMaxOutputIndex.hasChanged()){
            this.indexConfig.closedLoop.
            p(kPIndex.getNumber())
            .i(kIIndex.getNumber())
            .d(kDIndex.getNumber())
            .minOutput(kMinOutputIndex.getNumber())
            .maxOutput(kMaxOutputIndex.getNumber());
        }

    }
// Singleton
    public static Shooter getInstance(){
        if(instance == null) instance = new Shooter();
        return instance;
    }


}

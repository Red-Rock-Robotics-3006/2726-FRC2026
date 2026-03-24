package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import org.ejml.dense.row.decomposition.eig.watched.WatchedDoubleStepQREigen_DDRM;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.vision.LimelightHelpers;
import redrocklib.logging.SmartDashboardBoolean;
import redrocklib.logging.SmartDashboardNumber;
import redrocklib.wrappers.RedRockTalon;


public class Shooter extends SubsystemBase{
    private static Shooter instance = null;

    private Tank tank = Tank.getInstance();

    private final RedRockTalon shooterMotor1 = new RedRockTalon(31, "Shooter/shooterMotor1");
    private final RedRockTalon shooterMotor2 = new RedRockTalon(32, "Shooter/shooterMotor2");
    private final SparkFlex indexMotor = new SparkFlex(33, MotorType.kBrushless);

    private SparkFlexConfig indexConfig = new SparkFlexConfig();
    private SparkClosedLoopController indexController = indexMotor.getClosedLoopController();

    InterpolatingDoubleTreeMap table = InterpolatingDoubleTreeMap.ofEntries(Map.entry(5.0, 0.0)); //TODO put stuff in it

    private SmartDashboardNumber hubShooterSpeed = new SmartDashboardNumber("Shooter/hubShooterSpeed", 1270); //TODO
    private SmartDashboardNumber awayHubShooterSpeed = new SmartDashboardNumber("Shooter/awayHubShooterSpeed", 1400); //TODO
    private SmartDashboardNumber backwardsShooterSpeed = new SmartDashboardNumber("Shooter/backwardsShooterSpeed", 500); //TODO
    private SmartDashboardNumber speedUpSec = new SmartDashboardNumber("Shooter/speed-up-seconds", 2.5);
    private SmartDashboardNumber indexSpeed = new SmartDashboardNumber("Shooter/indexSpeed", 0.5); //TODO
    private SmartDashboardNumber indexKp = new SmartDashboardNumber("Shooter/indexKp", 0.5); //TODO
    private SmartDashboardNumber indexKi = new SmartDashboardNumber("Shooter/indexKi", 0); //TODO
    private SmartDashboardNumber indexKd = new SmartDashboardNumber("Shooter/indexKd", 0); //TODO
    private SmartDashboardBoolean isAtShooterSpeed = new SmartDashboardBoolean("Shooter/is-at-shooter-speed",false);

    private boolean autoShootActive = false;
    private SmartDashboardNumber shooterSpeedThreshold = new SmartDashboardNumber("Shooter/shooter-speed-threshold", 10);
    private Shooter(){
        super("Shooter");
        AutoLogOutputManager.addObject(this);
        this.shooterMotor1.withMotorOutputConfigs(
            new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withPeakForwardDutyCycle(1d)
            .withPeakReverseDutyCycle(-1d)
            .withNeutralMode(NeutralModeValue.Coast)
        )
        .withSlot0Configs(
            new Slot0Configs()
            .withKA(1) //TODO
            .withKS(0) //TODO
            .withKV(1) //TODO
            .withKP(0.5) //TODO
            .withKI(0) //TODO
            .withKD(0) //TODO 
        )
        .withMotionMagicConfigs(
            new MotionMagicConfigs()
            .withMotionMagicAcceleration(850)
            .withMotionMagicCruiseVelocity(150)
            .withMotionMagicJerk(10000000)
        )
        .withSpikeThreshold(17)
        .withCurrentLimitConfigs(
            new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(45)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(60)
            .withStatorCurrentLimitEnable(true)
        ).withTuningEnabled(false);
        
        this.shooterMotor2.withMotorOutputConfigs(
            new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withPeakForwardDutyCycle(1d)
            .withPeakReverseDutyCycle(-1d)
            .withNeutralMode(NeutralModeValue.Coast)
        )
        .withSlot0Configs(
            new Slot0Configs()
            .withKA(1) //TODO
            .withKS(0) //TODO
            .withKV(1) //TODO
            .withKP(0.5) //TODO
            .withKI(0) //TODO
            .withKD(0) //TODO 
        )
        .withMotionMagicConfigs(
            new MotionMagicConfigs()
            .withMotionMagicAcceleration(850)
            .withMotionMagicCruiseVelocity(150)
            .withMotionMagicJerk(10000000)
        )
        .withSpikeThreshold(17)
        .withCurrentLimitConfigs(
            new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(45)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(60)
            .withStatorCurrentLimitEnable(true)
        ).withTuningEnabled(false);

        indexConfig.idleMode(IdleMode.kBrake);
        indexConfig.closedLoop
        .p(indexKp.getNumber())
        .i(indexKi.getNumber())
        .d(indexKd.getNumber());
        indexMotor.configure(indexConfig, com.revrobotics.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void setShooterSpeedRPM(double RPM){
        this.shooterMotor1.setMotionMagicVelocity(RPM);
        this.shooterMotor2.setMotionMagicVelocity(RPM);
        SmartDashboard.putNumber("Shooter/rpm", RPM);
        Logger.recordOutput("Shooter/shooterMotor1TargetVelocity", RPM );
        Logger.recordOutput("Shooter/shooterMotor2TargetVelocity", RPM );
    }
    
    private void stopShooter(){
        this.shooterMotor1.motor.set(0);
        this.shooterMotor2.motor.set(0);
        Logger.recordOutput("Shooter/shooterMotor1TargetVelocity", 0 );
        Logger.recordOutput("Shooter/shooterMotor2TargetVelocity", 0 );
    }

    private void setIndexSpeed(){
        // this.indexController.setSetpoint(this.indexSpeed.getNumber(), ControlType.kVelocity);
        this.indexMotor.set(indexSpeed.getNumber());
        SmartDashboard.putNumber("Shooter/index-live-speed", this.indexMotor.getEncoder().getVelocity());
        Logger.recordOutput("Shooter/indexMotorTargetVelocity", this.indexSpeed.getNumber());
    }
    
    private void setIndexBackwardSpeed(){
        this.indexController.setSetpoint(-this.indexSpeed.getNumber(), ControlType.kVelocity);
        Logger.recordOutput("Shooter/indexMotorTargetVelocity", -this.indexSpeed.getNumber());
        
    }
    
    private void stopIndexer(){
        this.indexMotor.set(0);
        Logger.recordOutput("Shooter/indexMotorTargetVelocity", 0);
    }

    private double getShooterSpeed(double distance){
        return this.table.get(distance);
    }

    public boolean isAtShooterSpeed(){ //Checks if shooter is up to speed
        double shooterSpeed = this.getShooterSpeed(tank.distanceFromHub());
        return (Math.abs(shooterSpeed - 
        shooterMotor1.motor.getVelocity().getValueAsDouble()*60.0)) <= shooterSpeedThreshold.getNumber(); //getVelocity() is in RPS so convert to RPM
    }

    public boolean isAtAwayHubShooterSpeed(){ //Checks if shooter is up to speed
        return isAtShooterSpeed.getValue(); //getVelocity() is in RPS so convert to RPM
    }

    public Command autoAimShootCommand(){ //auto aligns tank and shoots
        return Commands.parallel(
            Commands.runOnce(() -> this.autoShootActive = true, this),
            tank.turnToHubCommand()
        );
    }

    public Command stopShooterCommand(){
        return Commands.sequence(
            Commands.runOnce(() -> this.autoShootActive = false),
            Commands.runOnce(() -> this.stopShooter(), this),
            Commands.runOnce(() -> this.stopIndexer(), this)
        );
    }

    public Command backwardShootCommand(){
        return Commands.sequence(
            Commands.runOnce(() -> this.setShooterSpeedRPM(backwardsShooterSpeed.getNumber()), this),
            Commands.runOnce(() -> this.setIndexBackwardSpeed(), this)
        );
    }

    public Command shootCommandHub(){ //Shoot from specific place on field
        return Commands.sequence(
            Commands.runOnce(() -> this.setShooterSpeedRPM(this.hubShooterSpeed.getNumber()), this),
            Commands.waitSeconds(this.speedUpSec.getNumber()),
            Commands.runOnce(() -> this.setIndexSpeed(), this)
        ); 
    }
    
    public Command shootCommandAwayHub(){
        return Commands.sequence(
            Commands.runOnce(() -> this.setShooterSpeedRPM(this.awayHubShooterSpeed.getNumber()), this),
            Commands.waitSeconds(this.speedUpSec.getNumber()),
            // Commands.waitUntil(() -> this.isAtShooterSpeed()),
            Commands.runOnce(() -> this.setIndexSpeed(), this)
        ); 
    }

    // public Command shootCommandAwayHub(){
    //     return new FunctionalCommand(
    //         () -> this.setShooterSpeedRPM(this.awayHubShooterSpeed.getNumber()), 
    //         () -> {
    //         }, 
    //         (interrupted) -> {
    //             this.setIndexSpeed();
    //         }, 
    //         () -> this.isAtAwayHubShooterSpeed(), 
    //         this
    //     );
    //     new Par
    // }

    public Command feedBackward(){
        return Commands.runOnce(() -> this.setIndexBackwardSpeed());
    }

    @Override
    public void periodic(){
        shooterMotor1.update();
        shooterMotor2.update();
        if(indexKp.hasChanged()
        || indexKi.hasChanged()
        || indexKp.hasChanged()){
            indexConfig.closedLoop
                .p(indexKd.getNumber())
                .i(indexKi.getNumber())
                .d(indexKd.getNumber());
        }

        if(autoShootActive){
            this.setShooterSpeedRPM((int)this.getShooterSpeed(tank.distanceFromHub()));
            if(this.isAtShooterSpeed())
                this.setIndexSpeed();
        }
        SmartDashboard.putNumber("Shooter/indexer-current-position", indexMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Shooter/indexer-current-velocity", indexMotor.getEncoder().getVelocity());
        SmartDashboard.putBoolean("Shooter/sees-hub-tag", tank.seesTag());
        this.isAtShooterSpeed.putBoolean((Math.abs(this.awayHubShooterSpeed.getNumber() - 
            shooterMotor1.motor.getVelocity().getValueAsDouble()*60.0)) <= shooterSpeedThreshold.getNumber());
        // SmartDashboard.putNumberArray("shooter/distance-from-hub-inch", distFromHubInch);
        // SmartDashboard.putNumberArray("shooter/shooter-speeds-rpm", shootSpeedsRPM);
    }

    public static Shooter getInstance(){
        if(instance == null) instance = new Shooter();
        return instance;
    }
}

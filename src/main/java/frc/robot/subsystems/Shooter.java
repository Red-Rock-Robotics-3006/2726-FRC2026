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
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
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
import redrocklib.wrappers.RedRockTalon;
import redrocklib.wrappers.logging.SmartDashboardBoolean;
import redrocklib.wrappers.logging.SmartDashboardNumber;


public class Shooter extends SubsystemBase{
    private static Shooter instance = null;

    private Tank tank = Tank.getInstance();

    private final RedRockTalon shooterMotor1 = new RedRockTalon(31, "Shooter/shooterMotor1");
    private final RedRockTalon shooterMotor2 = new RedRockTalon(32, "Shooter/shooterMotor2");
    private final SparkFlex indexMotor = new SparkFlex(33, MotorType.kBrushless);

    private SparkFlexConfig indexConfig = new SparkFlexConfig();
    private SparkClosedLoopController indexController = indexMotor.getClosedLoopController();

    InterpolatingDoubleTreeMap table = InterpolatingDoubleTreeMap.ofEntries(
        Map.entry(2.206, 2815.0), 
        Map.entry(2.436, 2915.0), 
        Map.entry(2.655, 3030.0), 
        Map.entry(2.973, 3216.0), 
        Map.entry(3.44, 3340.0),
        Map.entry(3.85, 3470.0),
        Map.entry(4.22, 3580.0),
        Map.entry(4.81, 3720.0),
        Map.entry(5.2, 3870.0)
    ); 

    private double targetRPM = 0.0;

    private SmartDashboardNumber hubShooterSpeed = new SmartDashboardNumber("Shooter/hubShooterSpeed", 3216); //TODO
    private SmartDashboardNumber lobShooterSpeed = new SmartDashboardNumber("Shooter/lobShooterSpeed", 5500); //TODO
    private SmartDashboardNumber awayHubShooterSpeed = new SmartDashboardNumber("Shooter/awayHubShooterSpeed", 3200); //TODO
    private SmartDashboardNumber backwardsShooterSpeed = new SmartDashboardNumber("Shooter/backwardsShooterSpeed", -1000); //TODO
    private SmartDashboardNumber speedUpSec = new SmartDashboardNumber("Shooter/speed-up-seconds", 2.5);
    private SmartDashboardNumber speedUpSecLob = new SmartDashboardNumber("Shooter/speed-up-seconds-lob", 0.7);
    private SmartDashboardNumber indexSpeed = new SmartDashboardNumber("Shooter/indexSpeed", 0.5); //TODO
    private SmartDashboardNumber indexKp = new SmartDashboardNumber("Shooter/indexKp", 0.5); //TODO
    private SmartDashboardNumber indexKi = new SmartDashboardNumber("Shooter/indexKi", 0); //TODO
    private SmartDashboardNumber indexKd = new SmartDashboardNumber("Shooter/indexKd", 0); //TODO
    private SmartDashboardBoolean isAtShooterSpeed = new SmartDashboardBoolean("Shooter/is-at-shooter-speed",false);
    private SmartDashboardNumber shootCommandThreshold = new SmartDashboardNumber("Shooter/shoot-command-threshold", 2.4);
    private SmartDashboardNumber lerpOffset = new SmartDashboardNumber("Shooter/lerp-offset", -5);
    private SmartDashboardNumber shooterWaitSeconds = new SmartDashboardNumber("Shooter/shooter-wait-seconds", 1);
    private boolean isShooting = false;
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
            .withKA(0.4) //TODO
            .withKS(0.15) //TODO
            .withKV(0.111) //TODO
            .withKP(0.6) //TODO
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
        ).withTuningEnabled(true);
        
        this.shooterMotor2.withMotorOutputConfigs(
            new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withPeakForwardDutyCycle(1d)
            .withPeakReverseDutyCycle(-1d)
            .withNeutralMode(NeutralModeValue.Coast)
        )
        .withSlot0Configs(
            new Slot0Configs()
            .withKA(0.4) //TODO
            .withKS(0.15) //TODO
            .withKV(0.111) //TODO
            .withKP(0.6) //TODO
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
        ).withTuningEnabled(true);

        indexConfig.idleMode(IdleMode.kBrake);
        indexConfig.closedLoop
        .p(indexKp.getNumber())
        .i(indexKi.getNumber())
        .d(indexKd.getNumber());
        indexMotor.configure(indexConfig, com.revrobotics.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public boolean shooterShooting(){
        return this.isShooting;
    }

    private void setShooterSpeedRPM(double RPM){
        this.shooterMotor1.motor.setControl(new VelocityVoltage(RPM/60).withSlot(0).withOverrideBrakeDurNeutral(true));
        this.shooterMotor2.motor.setControl(new VelocityVoltage(RPM/60).withSlot(0).withOverrideBrakeDurNeutral(true));
        SmartDashboard.putNumber("Shooter/rpm", RPM);
        this.targetRPM = RPM;
        this.isShooting =targetRPM>0;
        Logger.recordOutput("Shooter/shooterMotor1TargetVelocity", RPM );
        Logger.recordOutput("Shooter/shooterMotor2TargetVelocity", RPM );
    }
    
    private void stopShooter(){
        this.shooterMotor1.motor.setControl(new CoastOut());
        this.shooterMotor2.motor.setControl(new CoastOut());
        Logger.recordOutput("Shooter/shooterMotor1TargetVelocity", 0 );
        Logger.recordOutput("Shooter/shooterMotor2TargetVelocity", 0 );
        this.isShooting = false;
    }

    private void setIndexSpeed(){
        // this.indexController.setSetpoint(this.indexSpeed.getNumber(), ControlType.kVelocity);
        this.indexMotor.set(indexSpeed.getNumber());
        SmartDashboard.putNumber("Shooter/index-live-speed", this.indexMotor.getEncoder().getVelocity());
        Logger.recordOutput("Shooter/indexMotorTargetVelocity", this.indexSpeed.getNumber());
    }
    
    private void setIndexBackwardSpeed(){
        this.indexMotor.set(-this.indexSpeed.getNumber());
        Logger.recordOutput("Shooter/indexMotorTargetVelocity", -this.indexSpeed.getNumber());
        
    }
    
    private void stopIndexer(){
        this.indexMotor.set(0);
        Logger.recordOutput("Shooter/indexMotorTargetVelocity", 0);
    }

    private double getShooterSpeed(double distance){
        return this.table.get(distance) + this.lerpOffset.getNumber();
    }

    public boolean isAtShooterSpeed(){ //Checks if shooter is up to speed
        // double shooterSpeed = this.getShooterSpeed(tank.distanceFromHub());
        return (Math.abs(this.targetRPM - 
        shooterMotor1.motor.getVelocity().getValueAsDouble()*60.0)) <= shooterSpeedThreshold.getNumber(); //getVelocity() is in RPS so convert to RPM
    }

    public boolean isAtAwayHubShooterSpeed(){ //Checks if shooter is up to speed
        return isAtShooterSpeed.getValue(); //getVelocity() is in RPS so convert to RPM
    }

    public Command wait3SecondsCommands(){
        return Commands.waitSeconds(3);
    }

    public Command autoAimShootCommand(){ //auto aligns tank and shoots
        return Commands.parallel(
            Commands.runOnce(() -> this.autoShootActive = true, this),
            tank.turnToAngleCommand()
        );
    }

    public Command autoShootCommand(){
        return Commands.sequence(
            Commands.runOnce(() ->  this.setShooterSpeedRPM((int)this.getShooterSpeed(tank.distanceFromHub())), this),
            Commands.waitUntil(() -> this.isAtShooterSpeed()),
            Commands.waitSeconds(this.shooterWaitSeconds.getNumber()),
            Commands.runOnce(() -> this.setIndexSpeed())
        );
        // return Commands.runOnce(() -> this.autoShootActive = true, this);
    }

    public Command stopShooterCommand(){
        return Commands.sequence(
            Commands.runOnce(() -> this.autoShootActive = false),
            Commands.runOnce(() -> this.stopShooter(), this),
            Commands.runOnce(() -> this.stopIndexer(), this)
        );
    }

    public Command startIndexerCommand(){
        return Commands.runOnce(() ->  this.setIndexSpeed(), this);
    }
    
    public Command stopIndexerCommand(){
        return Commands.runOnce(() ->  this.stopIndexer(), this);
    }
    public Command backwardShootCommand(){
        return Commands.sequence(
            Commands.runOnce(() -> this.setShooterSpeedRPM(-backwardsShooterSpeed.getNumber()), this),
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

    public Command shootLobCommand(){
         return Commands.sequence(
            Commands.runOnce(() -> this.setShooterSpeedRPM((int)this.lobShooterSpeed.getNumber()), this),
            Commands.waitUntil(() -> this.isAtShooterSpeed()),
            // Commands.waitSeconds(this.speedUpSecLob.getNumber()),
            Commands.runOnce(() -> this.setIndexSpeed(), this)
        ); 
    }

    public Command decideWhatShoot(){
        return tank.distanceFromHub()  <= shootCommandThreshold.getNumber()? shootCommandHub(): shootCommandAwayHub();
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
        Logger.recordOutput("Shooter/IndexerAppliedCurrent", this.indexMotor.getOutputCurrent());
        Logger.recordOutput("Shooter/IndexerAppliedVelocity-dot-set", this.indexMotor.getAppliedOutput());
        Logger.recordOutput("Shooter/IndexerVelocity", this.indexMotor.getEncoder().getVelocity());
        Logger.recordOutput("Shooter/ShooterAppliedCurrent", this.shooterMotor1.motor.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("Shooter/ShooterVelocity", this.shooterMotor1.motor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/indexer-current-velocity", indexMotor.getEncoder().getVelocity());
        SmartDashboard.putBoolean("Shooter/sees-hub-tag", tank.seesTag());
        SmartDashboard.putBoolean("Shooter/isShootingMotor1", this.shooterMotor1.motor.getVelocity().getValueAsDouble() > 300);
        SmartDashboard.putBoolean("Shooter/isShootingMotor2", this.shooterMotor2.motor.getVelocity().getValueAsDouble() > 300);
        this.isAtShooterSpeed.putBoolean((Math.abs(this.awayHubShooterSpeed.getNumber() - 
            shooterMotor1.motor.getVelocity().getValueAsDouble()*60.0)) <= shooterSpeedThreshold.getNumber());
        Logger.recordOutput("Shooter/AtShooterSpeed", this.isAtShooterSpeed.getValue());
        // SmartDashboard.putNumberArray("shooter/distance-from-hub-inch", distFromHubInch);
        // SmartDashboard.putNumberArray("shooter/shooter-speeds-rpm", shootSpeedsRPM);
    }

    public static Shooter getInstance(){
        if(instance == null) instance = new Shooter();
        return instance;
    }
}

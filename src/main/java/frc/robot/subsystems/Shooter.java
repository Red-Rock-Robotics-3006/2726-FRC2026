package frc.robot.subsystems;

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
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.vision.LimelightHelpers;
import redrocklib.logging.SmartDashboardBoolean;
import redrocklib.logging.SmartDashboardNumber;
import redrocklib.wrappers.RedRockTalon;


public class Shooter extends SubsystemBase{
    private static Shooter instance = null;

    private Tank tank = Tank.getInstance();

    private final RedRockTalon shooterMotor1 = new RedRockTalon(4, "Shooter/shooterMotor1");
    private final RedRockTalon shooterMotor2 = new RedRockTalon(5, "Shooter/shooterMotor2");
    private final SparkMax indexMotor = new SparkMax(0, MotorType.kBrushless);

    private SparkMaxConfig indexConfig = new SparkMaxConfig();
    private SparkClosedLoopController indexController = indexMotor.getClosedLoopController();

    private double[] distFromHubInch; //TODO
    private double[] shootSpeedsRPM; //TODO

    private SmartDashboardNumber hubShooterSpeed = new SmartDashboardNumber("shooter/hubShooterSpeed", 500); //TODO
    private SmartDashboardNumber awayHubShooterSpeed = new SmartDashboardNumber("shooter/awayHubShooterSpeed", 500); //TODO
    private SmartDashboardNumber indexSpeed = new SmartDashboardNumber("shooter/indexSpeed", 100); //TODO
    private SmartDashboardNumber indexKp = new SmartDashboardNumber("shooter/indexKp", 0); //TODO
    private SmartDashboardNumber indexKi = new SmartDashboardNumber("shooter/indexKi", 0); //TODO
    private SmartDashboardNumber indexKd = new SmartDashboardNumber("shooter/indexKd", 0); //TODO

    private boolean autoShootActive = false;
    private Shooter(){
        super("Shooter");
        AutoLogOutputManager.addObject(this);
        this.shooterMotor1.withMotorOutputConfigs(
            new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withPeakForwardDutyCycle(1d)
            .withPeakReverseDutyCycle(-1d)
            .withNeutralMode(NeutralModeValue.Coast)
        )
        .withSlot0Configs(
            new Slot0Configs()
            .withKA(0) //TODO
            .withKS(0) //TODO
            .withKV(0) //TODO
            .withKP(0) //TODO
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
            .withKA(0) //TODO
            .withKS(0) //TODO
            .withKV(0) //TODO
            .withKP(0) //TODO
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

    public void setShooterSpeedRPM(double RPM){
        this.shooterMotor1.setMotionMagicVelocity(RPM);
        this.shooterMotor2.setMotionMagicVelocity(RPM);
        Logger.recordOutput("Shooter/shooterMotor1TargetVelocity", RPM );
        Logger.recordOutput("Shooter/shooterMotor2TargetVelocity", RPM );
    }
    
    public void stopShooter(){
        this.shooterMotor1.setMotionMagicVelocity(0);
        this.shooterMotor2.setMotionMagicVelocity(0);
        Logger.recordOutput("Shooter/shooterMotor1TargetVelocity", 0 );
        Logger.recordOutput("Shooter/shooterMotor2TargetVelocity", 0 );
    }

    public void setIndexSpeed(){
        this.indexController.setSetpoint(this.indexSpeed.getNumber(), ControlType.kVelocity);
        Logger.recordOutput("Shooter/indexMotorTargetVelocity", this.indexSpeed.getNumber());
    }
    
    public void setIndexBackwardSpeed(){
        this.indexController.setSetpoint(-this.indexSpeed.getNumber(), ControlType.kVelocity);
        Logger.recordOutput("Shooter/indexMotorTargetVelocity", -this.indexSpeed.getNumber());
        
    }
    
    public void stopIndexer(){
        this.indexMotor.set(0);
        Logger.recordOutput("Shooter/indexMotorTargetVelocity", 0);
    }

    public double getShooterSpeed(double[] distance, double[] speeds){
        int idx1 = 0, idx2 = 0;
        if(tank.distanceFromHub() >= distance[distance.length-1]) return speeds[speeds.length-1];
        if(tank.distanceFromHub() <= distance[0]) return speeds[0];
        for(int i = 1; i < distance.length; i++){
            if(tank.distanceFromHub() <= distance[i]){
                idx2 = i;
                idx1 = i-1;
                //Uses y = y0 + (x - x0) * (y - y0)/ (x - x0) to interpolate data
                double speed = speeds[idx1] + (tank.distanceFromHub() - distance[idx1])*(speeds[idx2]-speeds[idx1])/(distance[idx2]-distance[idx1]);
                Logger.recordOutput("Shooter/AutoAimShooterSpeed",  speed);
                return speed;
            }
        }
        return 0;
    }

    public boolean isAtShooterSpeed(){ //Checks if shooter is up to speed
        return (Math.abs(this.getShooterSpeed(distFromHubInch, shootSpeedsRPM) - 
        shooterMotor1.motor.getVelocity().getValueAsDouble()*60.0)) <= 100 && (Math.abs(this.getShooterSpeed(distFromHubInch, shootSpeedsRPM) - 
        shooterMotor2.motor.getVelocity().getValueAsDouble()*60.0)) <= 100; //getVelocity() is in RPS so convert to RPM
    }

    // public Command autoAimshootCommand(){ //Uses auto aim to shoot
    //     return Commands.sequence(
    //         Commands.runOnce(() -> this.setShooterSpeedRPM((int)this.getShooterSpeed(distFromHubInch, shootSpeedsRPM)), this), //TODO
    //         Commands.waitUntil(() -> this.isAtShooterSpeed()),
    //         Commands.runOnce(()-> this.setIndexSpeed())
    //     );
    // }

    public Command autoAimShootCommand(){ //auto aligns tank and shoots
        return Commands.sequence( 
            Commands.parallel(
                new FunctionalCommand(
                    () -> {
                        autoShootActive = true;
                    }, 
                    () -> {
                    }, 
                    (interrupted) -> {
                        this.autoShootActive = false;
                    }, 
                    () -> this.isAtShooterSpeed(),
                    this
                ),
                tank.turnToHubCommand()
            ),
            Commands.runOnce(() -> this.setIndexSpeed())
        );
    }

    public Command shootCommandHub(){ //Shoot from specific place on field
        return Commands.sequence(
            Commands.runOnce(() -> this.setShooterSpeedRPM(this.hubShooterSpeed.getNumber())),
            Commands.waitUntil(() -> this.isAtShooterSpeed()),
            Commands.runOnce(() -> this.setShooterSpeedRPM(this.hubShooterSpeed.getNumber()))
        ); 
    }
    
    public Command shootCommandAwayHub(){
        return Commands.sequence(
            Commands.runOnce(() -> this.setShooterSpeedRPM(this.awayHubShooterSpeed.getNumber())),
            Commands.waitUntil(() -> this.isAtShooterSpeed()),
            Commands.runOnce(() -> this.setShooterSpeedRPM(this.awayHubShooterSpeed.getNumber()))
        ); 
    }

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
            this.setShooterSpeedRPM((int)this.getShooterSpeed(distFromHubInch, shootSpeedsRPM));
        }
        SmartDashboard.putNumber("shooter/indexer-current-position", indexMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("shooter/indexer-current-velocity", indexMotor.getEncoder().getVelocity());
        SmartDashboard.putBoolean("shooter/sees-hub-tag", tank.seesTag());
        SmartDashboard.putNumberArray("shooter/distance-from-hub-inch", distFromHubInch);
        SmartDashboard.putNumberArray("shooter/shooter-speeds-rpm", shootSpeedsRPM);
    }

    public static Shooter getInstance(){
        if(instance == null) instance = new Shooter();
        return instance;
    }
}

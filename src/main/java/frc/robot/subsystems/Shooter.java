package frc.robot.subsystems;

import org.ejml.dense.row.decomposition.eig.watched.WatchedDoubleStepQREigen_DDRM;

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

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import redrocklib.logging.SmartDashboardBoolean;
import redrocklib.logging.SmartDashboardNumber;
import redrocklib.wrappers.RedRockTalon;


public class Shooter extends SubsystemBase{
    private static Shooter instance = null;

    private Tank tank = Tank.getInstance();
    private Conveyor conveyor = Conveyor.getInstance();

    private final RedRockTalon shooterMotor1 = new RedRockTalon(0, "Shooter/shooterMotor1");
    private final RedRockTalon shooterMotor2 = new RedRockTalon(0, "Shooter/shooterMotor2");

    private SmartDashboardNumber indexKp = new SmartDashboardNumber("shooter/indexKp", 0); //TODO
    private SmartDashboardNumber indexKi = new SmartDashboardNumber("shooter/indexKi", 0); //TODO
    private SmartDashboardNumber indexKd = new SmartDashboardNumber("shooter/indexKd", 0); //TODO

    private Shooter(){
        super("Shooter");
                
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

        // indexConfig.idleMode(IdleMode.kBrake);
        // indexConfig.closedLoop
        // .p(indexKp.getNumber())
        // .i(indexKi.getNumber())
        // .d(indexKd.getNumber());
        // indexMotor.configure(indexConfig, com.revrobotics.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setShooterSpeedRPM(int RPM){
        this.shooterMotor1.setMotionMagicVelocity(RPM);
        this.shooterMotor2.setMotionMagicVelocity(RPM);
    }

    public void stopShooter(){
        this.shooterMotor1.setMotionMagicVelocity(0);
        this.shooterMotor2.setMotionMagicVelocity(0);
    }

    //Returns speed from bestfit curve
    public double getSpeed(double x){//x distance from hub
        return Math.sqrt(x); //TODO change bestfit equation
    }

    public boolean isAtShooterSpeed(){ //Checks if shooter is up to speed
        return (Math.abs(this.getSpeed(tank.getRobotPose().getX()) - 
        shooterMotor1.motor.getVelocity().getValueAsDouble()*60.0)) <= 100 && (Math.abs(this.getSpeed(tank.getRobotPose().getX()) - 
        shooterMotor2.motor.getVelocity().getValueAsDouble()*60.0)) <= 100; //getVelocity() is in RPS so convert to RPM
    }

    public Command shootIndexCommand(){
        return Commands.sequence(
            Commands.runOnce(() -> this.setShooterSpeedRPM((int)this.getSpeed(tank.getXDistanceFromHub())), this), //TODO
            Commands.waitUntil(() -> this.isAtShooterSpeed()),
            conveyor.feedConveyorForwardCommand()
        );
    }

    public Command shootCommand(){
        return Commands.runOnce(() -> this.setShooterSpeedRPM((int)this.getSpeed(tank.getXDistanceFromHub())), this); //TODO
    }

    // public Command autoAlignShootCommand(){
    //     autoAimActive.setDefaultValue(true);
    //     return Commands.sequence(
    //         Commands.runOnce(() -> led.setLightsBlink(WHITE, 1)),
    //         tank.turnCommand(),
    //         this.shootCommand(),
    //         Commands.parallel(
    //             Commands.waitUntil(() -> tank.isAtAngle()),
    //             Commands.waitUntil(() -> this.isAtShooterSpeed())
    //         ),
    //         Commands.runOnce(() -> led.setLights(BLUE)),
    //         Commands.runOnce(() -> conveyor.feedConveyorForwardCommand())
    //     );
    // }
    @Override
    public void periodic(){
        shooterMotor1.update();
        shooterMotor2.update();
    }

    public static Shooter getInstance(){
        if(instance == null) instance = new Shooter();
        return instance;
    }
}

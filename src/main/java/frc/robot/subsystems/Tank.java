package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.vision.LimelightHelpers;
import redrocklib.logging.SmartDashboardBoolean;
import redrocklib.logging.SmartDashboardNumber;

public class Tank extends SubsystemBase{
    private static Tank instance = null;

    private SparkMax rightFront = new SparkMax(6, MotorType.kBrushless);
    private SparkMax rightBack = new SparkMax(7, MotorType.kBrushless);
    private SparkMax leftFront = new SparkMax(8, MotorType.kBrushless);
    private SparkMax leftBack = new SparkMax(9, MotorType.kBrushless);

    private String[] limelights = {"limelight1", "limelight2", "limelight3"};
    private double[] limelightHeights = {0, 0, 0}; //TODO vertical height off ground inch
    private double[] limelightAngles = {90, 90, 90}; //TODO angle of limelight degrees from vertical
    private int[] hubTags; //TODO go to limelight Point of Intrest tab and set the offsets so Tx is 0 at middle of hub
    
    private DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(22.5); //TODO
    
    private SmartDashboardNumber maxDrive = new SmartDashboardNumber("Tank/maxDrive", 0.3);
    private SmartDashboardNumber maxTurn = new SmartDashboardNumber("Tank/maxTurn", 0.3);
    private SmartDashboardNumber turnKp = new SmartDashboardNumber("Tank/turnSpeed", 0.1); //TODO
    private SmartDashboardNumber turnKi = new SmartDashboardNumber("Tank/turnSpeed", 0); //TODO
    private SmartDashboardNumber turnKd = new SmartDashboardNumber("Tank/turnSpeed", 0); //TODO
    
    private PIDController alignPID = new PIDController(turnKp.getNumber(), turnKi.getNumber(), turnKd.getNumber());

    private SmartDashboardBoolean onRedAlliance = new SmartDashboardBoolean("Tank/ontRedAlliance", false); //TODO
    
    private Tank(){
        super("Tank");

        this.hubTags = onRedAlliance.getValue()? new int[]{5, 8, 9, 10, 11, 2}:  new int[]{18, 27, 26, 25, 24};
        for(String limelight: limelights){
            LimelightHelpers.SetFiducialIDFiltersOverride(limelight, hubTags);
        }
        
        alignPID.setTolerance(5.0); //5 degrees

        SparkMaxConfig rightConfig = new SparkMaxConfig();
        SparkMaxConfig leftConfig = new SparkMaxConfig();

        rightConfig.idleMode(IdleMode.kBrake).inverted(false);
        leftConfig.idleMode(IdleMode.kBrake).inverted(true);

        rightFront.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightBack.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        leftFront.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        leftBack.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void driveRaw(double drive, double turn){
        rightFront.set(turn + drive);
        rightBack.set(turn + drive);
        leftFront.set(drive - turn);
        leftBack.set(drive - turn);
    }

    
    public void drive(double drive, double turn){
        this.driveRaw(drive * maxDrive.getNumber(), turn * maxTurn.getNumber());
    }

    //returns which limelight sees april tag on hub
    public String getLimelight(){
        for(String limelight: this.limelights){
            if(LimelightHelpers.getTV(limelight))
                return limelight;
        }
        return "";
    }

    //returns index in limelights array that sees april tag
    public int getLimelightIdx(){
        String limelight = getLimelight();
        return limelight.equals(this.limelights[0])? 0: limelight.equals(this.limelights[1])? 1: 2;
    }

    public boolean seesTag(){
        return !getLimelight().equals("");
    }

    //Turns till tx == 0
    public void turnToHub(){
        String limelight = getLimelight();
        if(!limelight.equals("")){
            drive(0, alignPID.calculate(LimelightHelpers.getTX(limelight), 0));
        }
    }


    public boolean isAtAngle(){
        return Math.abs(LimelightHelpers.getTX(getLimelight())) < 5; //TODO angle may be to small
    }

    //Uses the formula distance = (aprilTag height - mounted heigh)/ tan(mounting angle of limelight + angle to april tag)
    public double distanceFromHub(){
        String limelight = getLimelight();
        int idx = getLimelightIdx();
        if(LimelightHelpers.getTY(limelight) != 0 && !limelight.equals("")){
            return (44.25- this.limelightHeights[idx])/Math.tan(limelightAngles[idx] + LimelightHelpers.getTY(limelight));
        }
        return 0.0;
    }

    public Command turnToHubCommand(){
        return Commands.runOnce(() -> turnToHub());
    }

    @Override
    public void periodic(){
        if(turnKp.hasChanged()
        || turnKi.hasChanged()
        || turnKp.hasChanged()){
            alignPID.setPID(turnKp.getNumber(), turnKi.getNumber(), turnKd.getNumber());
        }
        SmartDashboard.putNumber("tank/right-front-current-position", rightFront.getEncoder().getPosition());
        SmartDashboard.putNumber("tank/right-front-current-velocity", rightFront.getEncoder().getVelocity());
        SmartDashboard.putNumber("tank/left-front-current-position", leftFront.getEncoder().getPosition());
        SmartDashboard.putNumber("tank/left-front-current-velocity", leftFront.getEncoder().getVelocity());
        SmartDashboard.putBoolean("tank/sees-hub-tag", seesTag());
        SmartDashboard.putNumberArray("tank/limelight-angles-from-vertical", this.limelightAngles);
        SmartDashboard.putNumberArray("tank/limelight-height-from-ground", this.limelightHeights);
        SmartDashboard.putStringArray("tank/limelights", this.limelights);
    }

    public static Tank getInstance(){
        if(instance == null) instance = new Tank();
        return instance;
    }
}

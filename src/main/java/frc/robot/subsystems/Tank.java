package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.vision.LimelightHelpers;
import frc.robot.vision.LimelightHelpers.PoseEstimate;
import redrocklib.logging.SmartDashboardBoolean;
import redrocklib.logging.SmartDashboardNumber;

public class Tank extends SubsystemBase{
    private static Tank instance = null;
    private SparkMax leftFront = new SparkMax(16, MotorType.kBrushless);
    private SparkMax rightFront = new SparkMax(16, MotorType.kBrushless);
    private SparkMax leftBack = new SparkMax(16, MotorType.kBrushless);
    private SparkMax rightBack = new SparkMax(16, MotorType.kBrushless);

    private String[] limelights = {"limelight1", "limelight2", "limelight3"};
    private double[][] limelightStdev = {{0.7 , 0.7, 999999999},{0.7 , 0.7, 999999999},{0.7 , 0.7, 999999999}};
    

    public SmartDashboardBoolean alliance = new SmartDashboardBoolean("Tank/alliance(true for red, false for blue)", false);    
    private Field2d field = new Field2d();
    private DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(22.5);
    private final Pigeon2 m_gyro = new Pigeon2(0); // TODO
    private RelativeEncoder m_rightEncoder = rightFront.getEncoder();
    private RelativeEncoder m_leftEncoder = leftFront.getEncoder();
    private final DifferentialDrivePoseEstimator m_poseEstimator = new DifferentialDrivePoseEstimator(
        m_kinematics, 
        m_gyro.getRotation2d(), 
        rightFront.getEncoder().getPosition(), 
        leftFront.getEncoder().getPosition(), 
        new Pose2d()
    );

    public static final double TURN_CONSTANT = 0;//TODO FILLER

    private SparkMaxConfig leftConf = new SparkMaxConfig();
    private SparkMaxConfig rightConf = new SparkMaxConfig();

    private SmartDashboardNumber driveMaxSpeed = new SmartDashboardNumber("Tank/driveMaxSpeed", 0.95);
    private SmartDashboardNumber turnMaxSpeed = new SmartDashboardNumber("Tank/turnMaxSpeed", 0.5);
    private SmartDashboardNumber autoaimVel = new SmartDashboardNumber("Tank/autoaimVelocity", 0.5);

    private Tank() {
        SparkMaxConfig leftConfig = new SparkMaxConfig();
        leftConfig.idleMode(IdleMode.kBrake).inverted(false);
        SparkMaxConfig rightConfig = new SparkMaxConfig();
        rightConfig.idleMode(IdleMode.kBrake).inverted(true);

        leftFront.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        leftBack.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightFront.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightBack.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void leftSetVel(double velocity){
        this.leftFront.set(velocity);
        this.leftBack.set(velocity);
    }
    public void rightSetVel(double velocity){
        this.rightFront.set(velocity);
        this.rightBack.set(velocity);
    }
    public void drive(double drivePercent, double turnPercent){
        this.leftSetVel(this.driveMaxSpeed.getNumber() * drivePercent + this.turnMaxSpeed.getNumber() * turnPercent);
        this.rightSetVel(this.driveMaxSpeed.getNumber() * drivePercent - this.turnMaxSpeed.getNumber() * turnPercent);
        
    }

    public boolean changeAngleDegrees(double theta){
        Pose2d startPose = this.getPose2d();
        if(theta < 0){
            this.leftSetVel(autoaimVel.getNumber());
            this.rightSetVel(-autoaimVel.getNumber());
        }
        else{
            this.leftSetVel(-autoaimVel.getNumber());
            this.rightSetVel(autoaimVel.getNumber());
            
        }
        

        //Implement checker to make sure you turn the correct angle
        if(this.getPose2d().getRotation().getDegrees() - startPose.getRotation().getDegrees() <=2)
            return true;
        return false;
    }
//

    public Pose2d getPose2d(){
        return this.m_poseEstimator.getEstimatedPosition();
    }

    public void updateOdometry(){
        m_poseEstimator.update(
            m_gyro.getRotation2d(),
            rightFront.getEncoder().getPosition(),
            leftFront.getEncoder().getPosition()
        );

        
        for(int i = 0; i < 3; i++){
            
            LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelights[i]);
            LimelightHelpers.SetRobotOrientation(this.limelights[i], m_gyro.getRotation2d().getDegrees(), m_gyro.getAngularVelocityXWorld().getValueAsDouble(), 0, 0, 0, 0);
            if((Math.abs(m_gyro.getAngularVelocityZWorld().getValueAsDouble()) <= 720 )&& !(mt2.tagCount == 0)){
                m_poseEstimator.addVisionMeasurement(mt2.pose, -mt2.latency/1000 + Utils.getCurrentTimeSeconds(), VecBuilder.fill(limelightStdev[i][0], limelightStdev[i][1], limelightStdev[i][2]));
                SmartDashboard.putBoolean("Limelight/" + limelights[i] + "/isAccepted", false);
            }
            else
                SmartDashboard.putBoolean("Limelight/" + limelights[i] + "/isAccepted", false);
            SmartDashboard.putBoolean("Limelight/" + limelights[i] + "/seesAprilTag", mt2.tagCount>0);
            
            
        }
        field.setRobotPose(m_poseEstimator.getEstimatedPosition());
        SmartDashboard.putData("Limelight/Pose", field);
    }

    public double getAngleToHub(){
        if(alliance.getValue()){
            return Math.atan2(11.92 - this.getPose2d().getY(), this.getPose2d().getX() - 4.033);
        }
        return Math.atan2(4.629 - this.getPose2d().getY(), 4.039 - this.getPose2d().getX())*180/Math.PI;
    }

    public double getAngleToLobbingPoint(){
        if(this.getPose2d().getY() < 4.039){
            if(alliance.getValue())
                return Math.atan2(1.9 - this.getPose2d().getY(), 14 - this.getPose2d().getX());    
            return Math.atan2(1.9 - this.getPose2d().getY(), 2.7 - this.getPose2d().getX());
        }
        if(alliance.getValue()){
            return Math.atan2(6-this.getPose2d().getY(), 14-this.getPose2d().getX());
        }
        return Math.atan2(6-this.getPose2d().getY(), this.getPose2d().getX()-2.7);
    }
    
    public void turnToHub(){
        this.changeAngleDegrees(this.getAngleToHub() - this.getPose2d().getRotation().getDegrees());
    }
    
    public void turnToLobPoint(){
        this.changeAngleDegrees(this.getAngleToHub() - this.getPose2d().getRotation().getDegrees());
    }

    public Command turnToHubCommand(){
        return Commands.runOnce(() -> this.turnToHub(), this);
    }

    public Command turnToLobPointCommand(){
        return Commands.runOnce(() -> this.turnToLobPoint(), this);
    }

    public Command autoAlignTankCommand(){
        if((this.getPose2d().getX() <= 4 && this.alliance.getValue() == false) || (this.getPose2d().getX() >= 12.6 && this.alliance.getValue() == true)){
            return this.turnToHubCommand();
        }
        return this.turnToLobPointCommand();
    }

    public Tank getInstance(){
        if(this.instance == null){
            instance = new Tank();
        }
        return this.instance;
    }
    @Override
    public void periodic(){
        instance.updateOdometry();
    }
    // public PoseEstimate getPose(){
    //     ;
    // }
    
}

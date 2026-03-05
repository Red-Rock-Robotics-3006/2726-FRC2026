package frc.robot.subsystems;

import java.util.Set;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.vision.LimelightHelpers;
import redrocklib.logging.SmartDashboardBoolean;
import redrocklib.logging.SmartDashboardNumber;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;

public class Tank extends SubsystemBase{
    private static Tank instance = null;

    private SparkMax rightFront = new SparkMax(6, MotorType.kBrushless);
    private SparkMax rightBack = new SparkMax(7, MotorType.kBrushless);
    private SparkMax leftFront = new SparkMax(8, MotorType.kBrushless);
    private SparkMax leftBack = new SparkMax(9, MotorType.kBrushless);

    private String[] limelights = {"limelight1", "limelight2", "limelight3"};
    private double[][] limelightStDev = {{0.7, 0.7, 99999}, {0.7, 0.7, 99999}, {0.7, 0.7, 99999}};

    private DifferentialDrivetrainSim m_driveSim = new DifferentialDrivetrainSim( 
        DCMotor.getNEO(4), //TODO motors
        7.0, //TODO The gear ratio between the motors and the wheels as output torque over input torque 
        7.5,  //TODO rotational inertia
        100.0, //TODO mass
        Units.inchesToMeters(3), //TODO radius of wheels(meters)
        Units.inchesToMeters(22),//TODO track width (meters)
        VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005) //TODO standard deviations
    );

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
    private Field2d fieldPose = new Field2d();

    private SmartDashboardNumber maxDrive = new SmartDashboardNumber("Tank/maxDrive", 0.3);
    private SmartDashboardNumber maxTurn = new SmartDashboardNumber("Tank/maxTurn", 0.3);
    private SmartDashboardNumber turnSpeed = new SmartDashboardNumber("Tank/turnSpeed", 0.1);

    private SmartDashboardBoolean redAlliance = new SmartDashboardBoolean("Tank/redAlliance", false); //TODO
    private SmartDashboardBoolean autoAimActive = new SmartDashboardBoolean("Tank/autoAimActive", false); //TODO


    private Tank(){
        super("Tank");

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
    
    //Turns to hub
    public void turnToHubAngle(){
        while(true){
            if(Math.abs(this.getAngleToHub() - this.getRobotPose().getRotation().getDegrees()) <= 3) //TODO checks if close to hub angle
                break;
            if(this.getAngleToHub() < 0)
                this.driveRaw(0, -turnSpeed.getNumber());
            else   
                this.driveRaw(0, turnSpeed.getNumber());
            }
        }
        
    //Turns to one of the four lobbing points
    public void turnToLobbingAngle(){
        while(true){
            if(Math.abs(this.getAngleToLobbingPoint() - this.getRobotPose().getRotation().getDegrees()) <= 3) //TODO checks if close to lobbing angle
                break;
            if(this.getAngleToHub() < 0)
                this.driveRaw(0, -turnSpeed.getNumber());
            else   
                this.driveRaw(0, turnSpeed.getNumber());
        }
    }
    
    //returns estimated pose in Pose2d which has (x in meters, y in meters, rotation)  
    public Pose2d getRobotPose(){
        return m_poseEstimator.getEstimatedPosition();
    }

    public double getXDistanceFromHub(){
        if(!redAlliance.getValue())
            return Math.abs(this.getRobotPose().getX() - 4.629);
        return Math.abs(this.getRobotPose().getX() - 11.92);
    }
    
    //returns angle(degrees) that robot needs to be at to face the hub
    public double getAngleToHub(){
        if(redAlliance.getValue()){
            return Math.atan2(11.92 - this.getRobotPose().getY(), this.getRobotPose().getX() - 4.033);
        }
        return Math.atan2(4.629 - this.getRobotPose().getY(), 4.039 - this.getRobotPose().getX())*180/Math.PI;
    }

    
    //Returns angle(degrees) to lobbing point (4 separate points uses pos and redAlliance boolean to find correct lobbing point)
    public double getAngleToLobbingPoint(){
        if(this.getRobotPose().getY() < 4.039){
            if(redAlliance.getValue())
                return Math.atan2(1.9 - this.getRobotPose().getY(), 14 - this.getRobotPose().getX());    
                return Math.atan2(1.9 - this.getRobotPose().getY(), 2.7 - this.getRobotPose().getX());
            }
            if(redAlliance.getValue()){
                return Math.atan2(6-this.getRobotPose().getY(), 14-this.getRobotPose().getX());
            }
            return Math.atan2(6-this.getRobotPose().getY(), this.getRobotPose().getX()-2.7);
    }

    //Checks if facing target
    public boolean isAtAngle(){
        return Math.abs(this.getAngleToHub()) <= 3 || Math.abs(this.getAngleToLobbingPoint()) <= 3; 
    }
        
    //updates poseEstimator, then checks if april tag is present, if so it updates poseEstimator with vision data
    public void updateOdometry(){
            m_poseEstimator.update(
            m_gyro.getRotation2d(),
            rightFront.getEncoder().getPosition(),
            leftFront.getEncoder().getPosition()
        );

        for(int i = 0;  i < limelights.length; i++){
            LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelights[i]);
            LimelightHelpers.SetRobotOrientation(limelights[i], m_gyro.getRotation2d().getDegrees(), m_gyro.getAngularVelocityZWorld().getValueAsDouble(), 0, 0, 0, 0);
            if(Math.abs(m_gyro.getAngularVelocityZWorld().getValueAsDouble()) <= 360 && mt2.tagCount != 0){
                m_poseEstimator.addVisionMeasurement(mt2.pose, Utils.getCurrentTimeSeconds() - mt2.latency/1000, VecBuilder.fill(limelightStDev[i][0], limelightStDev[i][1], limelightStDev[i][2]));
                SmartDashboard.putBoolean("Limelight/" + limelights[i] + "/isAccepted", true);
            }
            else{
                SmartDashboard.putBoolean("Limelight/" + limelights[i] + "isAccepted", false);
            }
            SmartDashboard.putBoolean("Limelight/" + limelights[i]+ "/seesAprilTag", mt2.tagCount > 0);
        }
        fieldPose.setRobotPose(m_poseEstimator.getEstimatedPosition());
        SmartDashboard.putData("Limelight/Pose", fieldPose);
    }

    public Command turnToHubCommand(){
        if(this.isAtAngle()){ 
            this.driveRaw(0, 0);
            return Commands.none();
        }
        return Commands.run(() -> this.turnToHubAngle(), this);
    }

    public Command turnToLobCommand(){
        if(this.isAtAngle()){ 
            this.driveRaw(0, 0);
            return Commands.none();
        }
        return Commands.run(() -> this.turnToLobbingAngle(), this);
    }

    //Turns to hub if in alliance zone turns to lobbing point if not (0-4 m blue zone, 12.6m on red zone)
    //uses defferd command to allow use of odometry at runtime and set.of(this) stops other uses of tank from running
    public Command turnCommand(){
        if((this.getRobotPose().getX() <= 4 && this.redAlliance.getValue() == false) || (this.getRobotPose().getX() >= 12.6 && this.redAlliance.getValue() == true))
                return this.turnToHubCommand();
            return this.turnToLobCommand();
    }

    @Override
    public void periodic(){
        instance.updateOdometry();
    }

    public static Tank getInstance(){
        if(instance == null) instance = new Tank();
        return instance;
    }
}

package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Utils;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.vision.LimelightHelpers;
import redrocklib.wrappers.logging.SmartDashboardBoolean;
import redrocklib.wrappers.logging.SmartDashboardNumber;

public class Tank extends SubsystemBase{
    private static Tank instance = null;

    private SparkMax rightFront = new SparkMax(11, MotorType.kBrushed);
    private SparkMax rightBack = new SparkMax(13, MotorType.kBrushed);
    private SparkMax leftFront = new SparkMax(12, MotorType.kBrushed);
    private SparkMax leftBack = new SparkMax(14, MotorType.kBrushed);

    private String limelightName = "limelight";
    private double limelightHeight = 0; //TODO vertical height off ground inch
    private double limelight = 90; //TODO angle of limelight degrees from vertical
    private double limelightAngle = 0.0;
    private int[] hubTags; //TODO go to limelight Point of Intrest tab and set the offsets so Tx is 0 at middle of hub
    
    private SmartDashboardNumber maxDrive = new SmartDashboardNumber("Tank/maxDrive", 0.8);
    private SmartDashboardNumber maxTurn = new SmartDashboardNumber("Tank/maxTurn", 0.8);
    private SmartDashboardNumber turnKp = new SmartDashboardNumber("Tank/Kp", 0.02); //TODO
    private SmartDashboardNumber turnKi = new SmartDashboardNumber("Tank/Ki", 0); //TODO
    private SmartDashboardNumber turnKd = new SmartDashboardNumber("Tank/Kd", 0.003); //TODO
    private SmartDashboardNumber turnKs = new SmartDashboardNumber("Tank/Ks", 0.15);

    private SmartDashboardNumber ambiguityThreshold = new SmartDashboardNumber("Limelight/ambiguity-threshold", 0.8);
    private SmartDashboardNumber distanceToCameraThreshold = new SmartDashboardNumber("Limelight/distance-to-camera-threshold", 5);
    
    private PIDController alignPID = new PIDController(turnKp.getNumber(), turnKi.getNumber(), turnKd.getNumber());
    private SmartDashboardNumber alignPIDTolerance = new SmartDashboardNumber("Tank/align-PID-tolerance", 5.0);
    private DriverStation.Alliance alliance = Alliance.Blue;
    
    private double[] limelightStDev = {0.7, 0.7, 0.8};
    private double[] encoderStDev = {0.5, 0.5, 0.7};
    private double[] visionStDev = {0.2, 0.2, 0.5};
    private DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(22.5);
    private double trackWidth = 22.0;
    private boolean doRejectUpdate = false;
    private LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
    private final Field2d field = new Field2d();
    private final Field2d limelightPos = new Field2d();
    private DifferentialDrivePoseEstimator m_poseEstimator = new DifferentialDrivePoseEstimator(
        m_kinematics, 
        new Rotation2d(), 
        0.0, 
        0.0, 
        new Pose2d(), 
        VecBuilder.fill(encoderStDev[0], encoderStDev[1], encoderStDev[2]),
        VecBuilder.fill(visionStDev[0], visionStDev[1], visionStDev[2])
    );

    private boolean autoAlignActive = false;
    private boolean turnSlowActive = false;
    private SmartDashboardNumber slowTurnMax = new SmartDashboardNumber("Tank/slow-turn-speed-max", 0.3);
    private SmartDashboardNumber isAtAngleThreshold = new SmartDashboardNumber("Tank/is-at-angle-threshold", 7);
    private double distanceFromHub = 0.0;
    private double angleToTarget = 0.0;
    private double robotAngle = 0.0;

    private enum RobotState{
        AutoAlign, 
        Driving
    }

    private RobotState state = RobotState.Driving;
    
    private SmartDashboardNumber redHubX = new SmartDashboardNumber("Tank/Points/red-hub-x", 4.629);
    private SmartDashboardNumber redHubY = new SmartDashboardNumber("Tank/Points/red-hub-y", 4.033);

    private SmartDashboardNumber blueHubX = new SmartDashboardNumber("Tank/Points/blue-hub-x", 11.92);
    private SmartDashboardNumber blueHubY = new SmartDashboardNumber("Tank/Points/blue-hub-y", 4.033);

    private SmartDashboardNumber blueLobLowX = new SmartDashboardNumber("Tank/Points/blue-lob-low-x", 14);
    private SmartDashboardNumber blueLobLowY = new SmartDashboardNumber("Tank/Points/blue-lob-low-y", 1.9);

    private SmartDashboardNumber blueLobHighX = new SmartDashboardNumber("Tank/Points/blue-lob-high-x", 14);
    private SmartDashboardNumber blueLobHighY = new SmartDashboardNumber("Tank/Points/blue-lob-high-y", 6);

    private SmartDashboardNumber redLobLowX = new SmartDashboardNumber("Tank/Points/red-lob-low-x", 2.7);
    private SmartDashboardNumber redLobLowY = new SmartDashboardNumber("Tank/Points/red-lob-low-y", 1.9);

    private SmartDashboardNumber redLobHighX = new SmartDashboardNumber("Tank/Points/red-lob-high-x", 2.7);
    private SmartDashboardNumber redLobHighY = new SmartDashboardNumber("Tank/Points/red-lob-high-y", 6);
    
    
    private Tank(){
        super("Tank");
        AutoLogOutputManager.addObject(this);
        alignPID.setTolerance(alignPIDTolerance.getNumber()); //5 degrees

        SparkMaxConfig rightConfig = new SparkMaxConfig();
        SparkMaxConfig leftConfig = new SparkMaxConfig();

        rightConfig.idleMode(IdleMode.kBrake).inverted(false);
        leftConfig.idleMode(IdleMode.kBrake).inverted(false);

        rightFront.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightBack.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        leftFront.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        leftBack.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void driveRaw(double drive, double turn){
        double right = -turn + drive;
        double left = -turn - drive;
        rightFront.set(right);
        rightBack.set(right);
        SmartDashboard.putNumber("Tank/right-set", right);
        Logger.recordOutput("Tank/Right", right);
        leftFront.set(left);
        leftBack.set(left);
        SmartDashboard.putNumber("Tank/left-set", left);
        Logger.recordOutput("Tank/Left", left);
    }

    
    public void drive(double drive, double turn){
        double driveValue = 0.0;
        double turnValue = 0.0;
        if(turnSlowActive){
            driveValue = drive * maxDrive.getNumber();
            turnValue = turn * slowTurnMax.getNumber();
            SmartDashboard.putNumber("Tank/drive-value-slow", driveValue);
            SmartDashboard.putNumber("Tank/drive-slow", drive);
            SmartDashboard.putNumber("Tank/trun-value-slow", turnValue);
            SmartDashboard.putNumber("Tank/turn-slow", turn);
            SmartDashboard.putBoolean("Tank/slow-turn-active", turnSlowActive);
            this.driveRaw(driveValue, turnValue);
        }
        else{
            driveValue = drive * maxDrive.getNumber();
            turnValue = turn * maxTurn.getNumber();
            SmartDashboard.putNumber("Tank/drive-value", driveValue);
            SmartDashboard.putNumber("Tank/drive", drive);
            SmartDashboard.putNumber("Tank/trun-value", turnValue);
            SmartDashboard.putNumber("Tank/turn", turn);
            SmartDashboard.putBoolean("Tank/slow-turn-active", turnSlowActive);
            this.driveRaw(driveValue, turnValue);
        }
    }

    public boolean isSlowActivated(){
        return turnSlowActive;
    }

    public void setSlowActive(boolean value){
        this.turnSlowActive = value;
    }

    // private int getLimelight(){
    //     for(int i = 0; i < this.limelights.length; i++){
    //         if(LimelightHelpers.getTV(limelights[i])){
    //             Logger.recordOutput("Tank/Limelights", limelights[i]);
    //             return i;
    //         }
    //     }
    //     return -1;
    // }

    private void setAllianceColor(DriverStation.Alliance alliance) {
        this.alliance = alliance;
    }

    // public boolean seesTag(){
    //     Logger.recordOutput("Tank/CanSeeLimeLights", getLimelight() > -1);
    //     return getLimelight() > -1;
    // }
    
    //Turns till tx == 0
    // private void turnToHub(){
        //     String limelight = this.limelights[getLimelight()];
        //     if(!limelight.equals("")){
            //         drive(0, alignPID.calculate(LimelightHelpers.getTX(limelight), 0));
            //         Logger.recordOutput("Tank/Status", "TURNING_TO_HUB");
            //     }
    // }
    
    
    // public boolean isAtAngle(){
        //     boolean atAngle =  getLimelight() != -1 && Math.abs(LimelightHelpers.getTX(this.limelights[getLimelight()])) < 5;
    //     Logger.recordOutput("Tank/AtTargetAngle", atAngle);
    //     return atAngle; //TODO angle may be to small
    // }
    
    public boolean seesTag(){
        boolean seesTag = LimelightHelpers.getTV("limelight");
        Logger.recordOutput("Tank/CanSeeLimeLights", seesTag);
        return seesTag;
    }

    //Checks if facing target
    public boolean isAtAngle(){
        return Math.abs(angleToTarget -getRobotPose().getRotation().getDegrees()) <= isAtAngleThreshold.getNumber(); 
    }
    
    //returns estimated pose in Pose2d which has (x in meters, y in meters, rotation)  
    public Pose2d getRobotPose(){
        return m_poseEstimator.getEstimatedPosition();
    }
    
    //returns angle(degrees) that robot needs to be at to face the hub
    public double getAngleToHub(){
        return  alliance == DriverStation.Alliance.Red
        ? Math.atan2(redHubY.getNumber() - this.getRobotPose().getY(), redHubX.getNumber() -this.getRobotPose().getX())*180/Math.PI 
        : Math.atan2(blueHubY.getNumber() - this.getRobotPose().getY(), blueHubX.getNumber() - this.getRobotPose().getX())*180/Math.PI;
        
    }

    public double getAngleToLob(){
        return this.getRobotPose().getY() < redHubY.getNumber() 
        ? (this.alliance == DriverStation.Alliance.Red
        ? Math.atan2(redLobLowY.getNumber() - this.getRobotPose().getY(), redLobLowX.getNumber() - this.getRobotPose().getX())*180/Math.PI
        : Math.atan2(blueLobLowY.getNumber() - this.getRobotPose().getY(), blueLobLowX.getNumber() - this.getRobotPose().getX())*180/Math.PI
        )
        :(this.alliance == DriverStation.Alliance.Red
            ? Math.atan2(redLobHighY.getNumber() - this.getRobotPose().getY(), redLobHighX.getNumber() -this.getRobotPose().getX())*180/Math.PI
            : Math.atan2(blueLobHighY.getNumber() - this.getRobotPose().getY(), blueLobHighX.getNumber() - this.getRobotPose().getX())*180/Math.PI
        );
    }

    //decides if hub turn or lob turn is better
    //The angle you need to turn to, to face the target(You must be at this angle to be facing the target)
    public double getAngleToTurn(){
        double angle =  (this.getRobotPose().getX() <= redHubX.getNumber() && this.alliance == DriverStation.Alliance.Red ||  this.getRobotPose().getX() >= blueHubX.getNumber() && this.alliance == DriverStation.Alliance.Blue
        ? getAngleToHub()
        : getAngleToLob());

        return angle < 0? angle +180: angle-180;
    }
    
    private void turnToAngle(){
        // if(angleToTarget - this.getRobotPose().getRotation().getDegrees())
        double turn = MathUtil.clamp(-alignPID.calculate((-angleToTarget+this.getRobotPose().getRotation().getDegrees()+540)%360 - 180, 0), -1+turnKs.getNumber(), 1-turnKs.getNumber());
        drive(0, turn + turnKs.getNumber()*Math.signum(turn));
        SmartDashboard.putNumber("Tank/clamp-turn-auto-align", turn);
        Logger.recordOutput("Tank/Status", "TURNING_TO_Angle");
    }
    
    // Uses pythagorian theorem
    public double distanceFromHub(){
        // this.distanceFromHub = alliance == DriverStation.Alliance.Red 
        // ? Math.abs(redHubX.getNumber() - this.getRobotPose().getX())
        // : Math.abs(blueHubX.getNumber() - this.getRobotPose().getX());  
        // return distanceFromHub;
        return alliance == DriverStation.Alliance.Red 
        ? Math.sqrt(Math.pow(2, redHubX.getNumber() - this.getRobotPose().getX()) + Math.pow(2, redHubY.getNumber() - this.getRobotPose().getY()))
        : Math.sqrt(Math.pow(2, blueHubX.getNumber() - this.getRobotPose().getX()) + Math.pow(2, blueHubY.getNumber() - this.getRobotPose().getY()));  
    }

    //Uses the formula distance = (aprilTag height - mounted heigh)/ tan(mounting angle of limelight + angle to april tag)
    // public double distanceFromHub(){
        // String limelight = getLimelight() == -1 ? this.limelights[getLimelight()];
        // int idx = getLimelight();
        // double res = 0;
        // if(LimelightHelpers.getTY(limelight) != 0 && !limelight.equals("")){
        //     res = (this.hubTagHeight- this.limelightHeights[idx])/Math.tan(limelightAngles[idx] + LimelightHelpers.getTY(limelight));
        // }
        // Logger.recordOutput("Tank/DistFromHub(not constant)", res);
        // return res;
        // return 0;
    // }

    public Command turnToAngleCommand(){
        return Commands.runOnce(() -> this.state = RobotState.AutoAlign, this);
    }

    public Command stopTurnToAngleCommand(){
        return Commands.runOnce(() -> this.state = RobotState.Driving, this);
    }

    //updates poseEstimator, then checks if april tag is present, if so it updates poseEstimator with vision data
    public void updateOdometry(){
        double rightMeters = rightFront.getEncoder().getPosition();
        double leftMeters = leftFront.getEncoder().getPosition();
        
        m_poseEstimator.update(
            new Rotation2d((rightMeters-leftMeters)/trackWidth), 
            leftMeters,
            rightMeters
        );

        mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);

        if(mt1.tagCount >= 1 && mt1.rawFiducials.length >= 1){
            doRejectUpdate = mt1.rawFiducials[0].ambiguity > ambiguityThreshold.getNumber() || mt1.rawFiducials[0].distToCamera > distanceToCameraThreshold.getNumber(); 
            
            SmartDashboard.putNumber("Limelight/ambiguity", mt1.rawFiducials[0].ambiguity);
            SmartDashboard.putNumber("Limelight/distance-to-camera", mt1.rawFiducials[0].distToCamera);
        }
        if(mt1.tagCount == 0) doRejectUpdate = true;
        if(!doRejectUpdate){
            m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(limelightStDev[0], limelightStDev[1], limelightStDev[2]));
            m_poseEstimator.addVisionMeasurement(mt1.pose, mt1.timestampSeconds);
        }
        SmartDashboard.putNumber("Limelight/limelight/timestamp-seconds", mt1.timestampSeconds);
        SmartDashboard.putNumber("Limelight/limelight/latency", mt1.latency);
        SmartDashboard.putNumber("Limelight/limelight/system-time-seconds", Utils.getSystemTimeSeconds());
        SmartDashboard.putNumber("Limelight/limelight/current-time-seconds", Utils.getCurrentTimeSeconds());
        SmartDashboard.putBoolean("Limelight/" + limelightName+ "/seesAprilTag", mt1.tagCount > 0);
        SmartDashboard.putBoolean("Limelight/is-at-angle", this.isAtAngle());
        Logger.recordOutput("Limelight/" + limelightName+ "/seesAprilTag", mt1.tagCount > 0);
        Logger.recordOutput("Limelight/" + limelightName + "/isAccepted", !doRejectUpdate);
        SmartDashboard.putBoolean("Limelight/" + limelightName + "/isAccepted", !doRejectUpdate);
}

    @Override
    public void periodic(){
        if(state == RobotState.Driving)
            this.drive(-RobotContainer.driveStick.getLeftY(), RobotContainer.driveStick.getRightX());
        else if(state == RobotState.AutoAlign && !this.isAtAngle()){
            this.turnToAngle();
            //RobotContainer.setRumble(this.isAtAngle()? 0.25: 0.0);
        }

        updateOdometry();
        field.setRobotPose(m_poseEstimator.getEstimatedPosition());
        limelightPos.setRobotPose(mt1.pose);
        SmartDashboard.putData("Tank/field-pos", field);

        angleToTarget = getAngleToTurn();
        robotAngle = getRobotPose().getRotation().getDegrees();
        SmartDashboard.putNumber("Tank/angle-to-target", angleToTarget);
        SmartDashboard.putNumber("Tank/current-robot-angle", robotAngle);
        SmartDashboard.putData("Limelight/limelight-pos", limelightPos);

        if(turnKp.hasChanged()
        || turnKi.hasChanged()
        || turnKd.hasChanged()){
            alignPID.setPID(turnKp.getNumber(), turnKi.getNumber(), turnKd.getNumber());
        }

        //else{
            //RobotContainer.setRumble(LimelightHelpers.getTV(limelightName)? 0.02: 0);
            //if(this.isAtAngle())
                //RobotContainer.setRumble(0.7);
        //}

        this.distanceFromHub = this.distanceFromHub();
        SmartDashboard.putNumber("Tank/distance-from-hub", distanceFromHub);

        if(DriverStation.isDisabled()){
            DriverStation.getAlliance().ifPresent(
                allianceColor -> {
                    this.setAllianceColor(allianceColor);
                    // if(allianceColor == Alliance.Blue){
                    //     this.hubTags = onRedAlliance.getValue()? new int[]{5, 8, 9, 10, 11, 2}:  new int[]{18, 27, 26, 25, 24};
                    //     for(String limelight: limelights)
                    //         LimelightHelpers.SetFiducialIDFiltersOverride(limelight, hubTags);
                    // }
                }
                );
            }
            SmartDashboard.putNumber("Tank/right-front-current-position", rightFront.getEncoder().getPosition());
            SmartDashboard.putNumber("Tank/right-front-current-velocity", rightFront.getEncoder().getVelocity());
            SmartDashboard.putNumber("Tank/left-front-current-position", leftFront.getEncoder().getPosition());
            SmartDashboard.putNumber("Tank/left-front-current-velocity", leftFront.getEncoder().getVelocity());
            SmartDashboard.putBoolean("Tank/sees-hub-tag", seesTag());
            SmartDashboard.putNumber("Tank/limelight-angles-from-vertical", this.limelightAngle);
            SmartDashboard.putNumber("Tank/limelight-height-from-ground", this.limelightHeight);
            SmartDashboard.putString("Tank/limelights", this.limelightName);
            for(double arr: limelightStDev){
                SmartDashboard.putNumber("Tank/limelight-stDev", arr);
            }
            
            Logger.recordOutput("Tank/limelight-angles-from-vertical", this.limelightAngle);
            Logger.recordOutput("Tank/limelight-height-from-ground", this.limelightHeight);
            Logger.recordOutput("Tank/limelights", this.limelightName);
            Logger.recordOutput("Tank/RightVelocity", rightFront.getEncoder().getVelocity());
            Logger.recordOutput("Tank/LeftVelocity", leftFront.getEncoder().getVelocity());
            Logger.recordOutput("Tank/RightFrontCurrent", rightFront.getOutputCurrent());
            Logger.recordOutput("Tank/RightBackCurrent", rightBack.getOutputCurrent());
            Logger.recordOutput("Tank/LeftFrontCurrent", leftFront.getOutputCurrent());
            Logger.recordOutput("Tank/LeftBackCurrent", leftBack.getOutputCurrent());
            Logger.recordOutput("Limelight/Position", m_poseEstimator.getEstimatedPosition());
        }

    public static Tank getInstance(){
        if(instance == null) instance = new Tank();
        return instance;
    }
}

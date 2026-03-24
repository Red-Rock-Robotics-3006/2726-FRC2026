package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import choreo.auto.AutoFactory;
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
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.vision.LimelightHelpers;
import redrocklib.logging.SmartDashboardBoolean;
import redrocklib.logging.SmartDashboardNumber;

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
    
    private SmartDashboardNumber maxDrive = new SmartDashboardNumber("Tank/maxDrive", 0.3);
    private SmartDashboardNumber maxTurn = new SmartDashboardNumber("Tank/maxTurn", 0.3);
    private SmartDashboardNumber turnKp = new SmartDashboardNumber("Tank/Kp", 0.05); //TODO
    private SmartDashboardNumber turnKi = new SmartDashboardNumber("Tank/Ki", 0); //TODO
    private SmartDashboardNumber turnKd = new SmartDashboardNumber("Tank/Kd", 0); //TODO
    
    private PIDController alignPID = new PIDController(turnKp.getNumber(), turnKi.getNumber(), turnKd.getNumber());
    private SmartDashboardNumber alignPIDTolerance = new SmartDashboardNumber("Tank/align-PID-tolerance", 5.0);
    private DriverStation.Alliance alliance = Alliance.Blue;
    
    private double[] limelightStDev = {0.7, 0.7, 99999};
    private double[] encoderStDev = {0.5, 0.5, 0.7};
    private double[] visionStDev = {0.2, 0.2, 0.5};
    private DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(22.5);
    private double trackWidth = 22.0;
    private boolean doRejectUpdate = false;
    private final Field2d field = new Field2d();
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
    private boolean rumbleBig = false;
    private SmartDashboardNumber isAtAngleThreshold = new SmartDashboardNumber("Tank/is-at-angle-threshold", 5);
    private double distanceFromHub = 0.0;

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

        SmartDashboard.putData("Tank/field-pos", field);
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
        double driveValue = drive * maxDrive.getNumber();
        double turnValue = turn * maxDrive.getNumber();
        SmartDashboard.putNumber("Tank/drive-value", driveValue);
        SmartDashboard.putNumber("Tank/drive", drive);
        SmartDashboard.putNumber("Tank/trun-value", turnValue);
        SmartDashboard.putNumber("Tank/turn", turn);
        this.driveRaw(driveValue, turnValue);
    }

    // //returns which limelight sees april tag on hub
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
        return Math.abs(this.getAngleToTurn()) <= isAtAngleThreshold.getNumber(); 
    }
    
    //returns estimated pose in Pose2d which has (x in meters, y in meters, rotation)  
    public Pose2d getRobotPose(){
        return m_poseEstimator.getEstimatedPosition();
    }
    
    //returns angle(degrees) that robot needs to be at to face the hub
    public double getAngleToHub(){
        return alliance == DriverStation.Alliance.Red
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
    public double getAngleToTurn(){
        return this.getRobotPose().getX() <= redHubX.getNumber() && this.alliance == DriverStation.Alliance.Red ||  this.getRobotPose().getX() >= blueHubX.getNumber() && this.alliance == DriverStation.Alliance.Blue
        ? getAngleToHub()
        : getAngleToLob();
    }
    
    private void turnToAngle(){
        drive(0, alignPID.calculate(this.getRobotPose().getRotation().getDegrees(), getAngleToTurn()));
        Logger.recordOutput("Tank/Status", "TURNING_TO_Angle");
    }
    
    // Uses pythagorian theorem
    public double distanceFromHub(){
        this.distanceFromHub = alliance == DriverStation.Alliance.Red 
        ? Math.abs(redHubX.getNumber() - this.getRobotPose().getX())
        : Math.abs(blueHubX.getNumber() - this.getRobotPose().getX());  
        return distanceFromHub;
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

    public Command turnToHubCommand(){
        return new FunctionalCommand(
            () -> {
                autoAlignActive = true;
            }, 
            () -> {
            },
            (interrupted) -> {
                autoAlignActive = false;
            },
            () -> this.isAtAngle(),
            this
        );
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

        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        if(mt1.tagCount == 1 && mt1.rawFiducials.length == 1){
            if(mt1.rawFiducials[0].ambiguity > 0.7 || mt1.rawFiducials[0].distToCamera > 4) doRejectUpdate = true; 
        }
        if(mt1.tagCount == 0) doRejectUpdate = true;
        if(!doRejectUpdate){
            m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(limelightStDev[0], limelightStDev[1], limelightStDev[2]));
            m_poseEstimator.addVisionMeasurement(mt1.pose, mt1.timestampSeconds);
        }
        SmartDashboard.putBoolean("Limelight/" + limelightName+ "/seesAprilTag", mt1.tagCount > 0);
        Logger.recordOutput("Limelight/" + limelightName+ "/seesAprilTag", mt1.tagCount > 0);
        Logger.recordOutput("Limelight/" + limelightName + "/isAccepted", !doRejectUpdate);
        SmartDashboard.putBoolean("Limelight/" + limelightName + "/isAccepted", !doRejectUpdate);
}

    @Override
    public void periodic(){
        updateOdometry();
        field.setRobotPose(m_poseEstimator.getEstimatedPosition());

        if(turnKp.hasChanged()
        || turnKi.hasChanged()
        || turnKp.hasChanged()){
            alignPID.setPID(turnKp.getNumber(), turnKi.getNumber(), turnKd.getNumber());
        }

        if(autoAlignActive){
            this.turnToAngle();
            RobotContainer.setRumble(this.isAtAngle()? 0: 0.8);
        }
        else{
            RobotContainer.setRumble(LimelightHelpers.getTV(limelightName)? 0.2: 0);
        }

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
    }

    public static Tank getInstance(){
        if(instance == null) instance = new Tank();
        return instance;
    }
}

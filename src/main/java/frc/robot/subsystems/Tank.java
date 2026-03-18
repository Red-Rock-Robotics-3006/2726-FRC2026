package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

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
    
    private SmartDashboardNumber maxDrive = new SmartDashboardNumber("Tank/maxDrive", 0.3);
    private SmartDashboardNumber maxTurn = new SmartDashboardNumber("Tank/maxTurn", 0.3);
    private SmartDashboardNumber turnKp = new SmartDashboardNumber("Tank/turnSpeed", 0.1); //TODO
    private SmartDashboardNumber turnKi = new SmartDashboardNumber("Tank/turnSpeed", 0); //TODO
    private SmartDashboardNumber turnKd = new SmartDashboardNumber("Tank/turnSpeed", 0); //TODO
    
    private PIDController alignPID = new PIDController(turnKp.getNumber(), turnKi.getNumber(), turnKd.getNumber());
    private double alignPIDTolerence = 5.0;
    private DriverStation.Alliance alliance = Alliance.Blue;
    
    private double[][] limelightStDev = {{0.7, 0.7, 99999}, {0.7, 0.7, 99999}, {0.7, 0.7, 99999}};
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

    private SmartDashboardBoolean onRedAlliance = new SmartDashboardBoolean("Tank/ontRedAlliance", false); //TODO
    private boolean autoAlignActive = false;
    private Double hubTagHeight = 44.25;
    
    private Tank(){
        super("Tank");
        
        alignPID.setTolerance(alignPIDTolerence); //5 degrees

        SparkMaxConfig rightConfig = new SparkMaxConfig();
        SparkMaxConfig leftConfig = new SparkMaxConfig();

        rightConfig.idleMode(IdleMode.kBrake).inverted(false);
        leftConfig.idleMode(IdleMode.kBrake).inverted(true);

        rightFront.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightBack.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        leftFront.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        leftBack.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SmartDashboard.putData("Tank/field-pos", field);
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
    private int getLimelight(){
        for(int i = 0; i < this.limelights.length; i++){
            if(LimelightHelpers.getTV(limelights[i]))
                return i;
        }
        return -1;
    }

    private void setAllianceColor(DriverStation.Alliance alliance) {
        this.alliance = alliance;
    }

    public boolean seesTag(){
        return getLimelight() > -1;
    }

    //Turns till tx == 0
    private void turnToHub(){
        String limelight = this.limelights[getLimelight()];
        if(!limelight.equals("")){
            drive(0, alignPID.calculate(LimelightHelpers.getTX(limelight), 0));
        }
    }


    public boolean isAtAngle(){
        return Math.abs(LimelightHelpers.getTX(this.limelights[getLimelight()])) < 5; //TODO angle may be to small
    }

    //Uses the formula distance = (aprilTag height - mounted heigh)/ tan(mounting angle of limelight + angle to april tag)
    public double distanceFromHub(){
        String limelight = this.limelights[getLimelight()];
        int idx = getLimelight();
        if(LimelightHelpers.getTY(limelight) != 0 && !limelight.equals("")){
            return (this.hubTagHeight- this.limelightHeights[idx])/Math.tan(limelightAngles[idx] + LimelightHelpers.getTY(limelight));
        }
        return 0.0;
    }

    public Command turnToHubCommand(){
        return new FunctionalCommand(
            () -> autoAlignActive = true, 
            () -> {
            },
            (interrupted) -> autoAlignActive = false,
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

        for(int i = 0;  i < limelights.length; i++){
            LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelights[i]);
            if(mt1.tagCount == 1 && mt1.rawFiducials.length == 1){
                if(mt1.rawFiducials[0].ambiguity > 0.7 || mt1.rawFiducials[0].distToCamera > 4) doRejectUpdate = true; 
            }
            if(mt1.tagCount == 0) doRejectUpdate = true;
            if(!doRejectUpdate){
                m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(limelightStDev[i][0], limelightStDev[i][1], limelightStDev[i][2]));
                m_poseEstimator.addVisionMeasurement(mt1.pose, mt1.timestampSeconds);
            }
            SmartDashboard.putBoolean("Limelight/" + limelights[i]+ "/seesAprilTag", mt1.tagCount > 0);
            SmartDashboard.putBoolean("Limelight/" + limelights[i] + "/isAccepted", !doRejectUpdate);
        }
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
            this.turnToHub();
        }

        if(DriverStation.isDisabled()){
            DriverStation.getAlliance().ifPresent(
                allianceColor -> {
                    this.setAllianceColor(allianceColor);
                    if(allianceColor == Alliance.Blue){
                        this.hubTags = onRedAlliance.getValue()? new int[]{5, 8, 9, 10, 11, 2}:  new int[]{18, 27, 26, 25, 24};
                        for(String limelight: limelights)
                            LimelightHelpers.SetFiducialIDFiltersOverride(limelight, hubTags);
                    }
                }
            );
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

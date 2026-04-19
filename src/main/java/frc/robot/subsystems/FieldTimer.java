package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FieldTimer extends SubsystemBase{
    private static Timer timer = new Timer();

    public FieldTimer(){
        super();
        resetTimer();
    }

    public static void resetTimer(){
        timer.reset();
        timer.start();
    }

    

    @Override
    public void periodic(){
        double matchTime;
        if(DriverStation.isFMSAttached()){
            matchTime = Timer.getMatchTime();
        }
        else matchTime = 140 - timer.get();

        String shift = "";

        // if (DriverStation.isAutonomousEnabled()){
        //     shift = "AUTO";
        //     SmartDashboard.putNumber("Field/Time Remaining In Shift", matchTime);
        // }

        if (DriverStation.isTeleopEnabled()) {
            if (matchTime > 130) {
                shift = "TRANSITION"; 
                SmartDashboard.putNumber("Field/Time Remaining In Shift", matchTime - 130);
            } else if (matchTime > 105) {
                shift = "SHIFT 1/4";
                SmartDashboard.putNumber("Field/Time Remaining In Shift", matchTime - 105);
            } else if (matchTime > 80) {
                shift = "SHIFT 2/4";
                SmartDashboard.putNumber("Field/Time Remaining In Shift", matchTime - 80);
            } else if (matchTime > 55) {
                shift = "SHIFT 3/4";
                SmartDashboard.putNumber("Field/Time Remaining In Shift", matchTime - 55);
            } else if (matchTime > 30) {
                shift = "SHIFT 4/4";
                SmartDashboard.putNumber("Field/Time Remaining In Shift", matchTime - 30);
            } else {
                shift = "ENDGAME";
                SmartDashboard.putNumber("Field/Time Remaining In Shift", matchTime);
            }
        } 

        else if (DriverStation.isDisabled())
            shift = "DISABLED";

        SmartDashboard.putString("Shift", shift);
        SmartDashboard.putNumber("Match Time Remaining", matchTime);
        if(DriverStation.isAutonomousEnabled())    
            SmartDashboard.putNumber("Match Time Remaining", matchTime+140);

    }
}
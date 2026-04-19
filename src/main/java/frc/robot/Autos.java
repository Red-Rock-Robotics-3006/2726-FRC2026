package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tank;
import redrocklib.wrappers.logging.SmartDashboardNumber;

public class Autos {

    private static Shooter shooter = Shooter.getInstance();
    private static Intake intake = Intake.getInstance();
    private static Tank tank = Tank.getInstance();

    private static SmartDashboardNumber shootSeconds = new SmartDashboardNumber("autos/shooter-seconds", 8);
    private static SmartDashboardNumber autoDriveSpeed = new SmartDashboardNumber("Autos/auto-drive-speed", 0.7);
    private static SmartDashboardNumber autoDriveSpeedDepot = new SmartDashboardNumber("Autos/auto-drive-speed", 0.7);
    private static SmartDashboardNumber autoDriveSpeedSlow = new SmartDashboardNumber("Autos/auto-drive-speed-slow", 0.5);
    private static SmartDashboardNumber autoTurnSpeed = new SmartDashboardNumber("Autos/auto-turn-speed", 0.8);
    private static SmartDashboardNumber turnSeconds = new SmartDashboardNumber("Autos/auto-turn-seconds", 0.4);
    private static SmartDashboardNumber driveSeconds = new SmartDashboardNumber("Autos/auto-drive-seconds", 3.5);
    private static SmartDashboardNumber driveSecondsDepot = new SmartDashboardNumber("Autos/auto-drive-seconds", 2);
    private static SmartDashboardNumber preloadSeconds = new SmartDashboardNumber("Autos/preload-wiat-seconds", 7);
    private static SmartDashboardNumber depotShootSeconds = new SmartDashboardNumber("Autos/depot-shoot-seconds", 7);
    private static SmartDashboardNumber waitSeconds = new SmartDashboardNumber("Autos/wait-seconds", 0);

    public static Command awayHubPreload(){
        return Commands.sequence(
            shooter.shootCommandAwayHub(),
            Commands.waitSeconds(shootSeconds.getNumber()),
            shooter.stopShooterCommand()
        );
    }

    public static Command hubPreload(){
        return Commands.sequence(
            shooter.shootCommandHub(),
            Commands.waitSeconds(shootSeconds.getNumber()),
            shooter.stopShooterCommand()
        );
    }
    
    public static Command preloadsZeroIntake(){
        return Commands.sequence(
            Commands.runOnce(() -> tank.setStateAuto()),
            intake.resetIntakeCommand(),
            shooter.shootAutonmousCommand(),
            Commands.waitSeconds(7),
            shooter.stopShooterCommand(),
            Commands.runOnce(() -> tank.setStateDriving())
        );
    }
    //shoots preloads goes to middle and intakes
    public static Command goToMiddle(){
        return Commands.sequence(
            Commands.runOnce(() -> tank.setStateAuto()),
            intake.resetIntakeCommand(),
            Commands.runOnce(() -> tank.setStateAutoAlign()),
            tank.turnToAngleCommand(),
            Commands.waitUntil(() -> tank.isAtAngle()),
            tank.stopTurnToAngleCommand(),
            shooter.autoShootCommand(),
            // shooter.shootCommandHub(),
            Commands.waitSeconds(preloadSeconds.getNumber()),
            shooter.stopShooterCommand(),
            Commands.runOnce(() -> tank.setStateAuto()),
            Commands.waitSeconds(waitSeconds.getNumber()),
            Commands.parallel(
                Commands.sequence(
                    intake.runIntakeCommand(),
                    Commands.waitSeconds(10),
                    intake.stopIntakeRollerCommand()
                ),
                Commands.sequence(
                    Commands.runOnce(() -> tank.drive(-autoDriveSpeedSlow.getNumber(), autoTurnSpeed.getNumber()), tank),
                    Commands.waitSeconds(turnSeconds.getNumber()),
                    Commands.runOnce(() -> tank.drive(0,0), tank),
                    Commands.runOnce(() -> tank.drive(-autoDriveSpeed.getNumber(), 0), tank),
                    Commands.waitSeconds(driveSeconds.getNumber()),
                    Commands.runOnce(() -> tank.drive(0,0), tank)
                )
                ),
                // intake.runIntakeCommand(),
                // Commands.runOnce(() -> tank.drive(autoTurnSpeed.getNumber(), 0), tank),
                // Commands.waitSeconds(turnSeconds.getNumber()),
                Commands.runOnce(() -> tank.setStateAutoAlign()),
                tank.turnToAngleCommand(),
                Commands.waitUntil(() -> tank.isAtAngle()),
                tank.stopTurnToAngleCommand(),
                shooter.autoShootCommand(),
            // Commands.runOnce(() -> tank.drive(0,0), tank)
            Commands.runOnce(() -> tank.setStateDriving())
        );
    }

    public static Command depotShoot(){
        return Commands.sequence(
            Commands.runOnce(() -> tank.setStateAuto()),
            intake.resetIntakeCommand(),
            Commands.parallel(
                Commands.sequence(
                    intake.runIntakeCommand(),
                    Commands.waitSeconds(7),
                    intake.stopIntakeRollerCommand()
                ),
                Commands.sequence(
                    Commands.runOnce(() -> tank.drive(autoDriveSpeed.getNumber(), 0), tank),
                    Commands.waitSeconds(driveSecondsDepot.getNumber()),
                    Commands.runOnce(() -> tank.drive(0,0), tank)
                )
            ),
            Commands.runOnce(() -> tank.setStateAutoAlign()),
            tank.turnToAngleCommand(),
            Commands.waitUntil(() -> tank.isAtAngle()),
            tank.stopTurnToAngleCommand(),
            Commands.runOnce(() -> tank.setStateAuto()),
            Commands.parallel(
                Commands.sequence(
                shooter.shootAutonmousCommandDepot(),
                Commands.waitSeconds(depotShootSeconds.getNumber()),
                shooter.stopShooterCommand()
                ),
                Commands.sequence(
                    Commands.waitSeconds(3),
                    intake.deployIntakeCommand(),
                    intake.stowIntakeCommand()
                )
            ),
            Commands.runOnce(() -> tank.setStateDriving())
        );
    }
}
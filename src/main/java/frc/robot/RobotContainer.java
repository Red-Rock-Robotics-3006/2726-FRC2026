// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.utility.WheelForceCalculator.Feedforwards;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
// import frc.robot.subsystems.LEDTest;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tank;

public class RobotContainer {

  public final static CommandXboxController driveStick = new CommandXboxController(0);
  // private final CommandXboxController mechStick = new CommandXboxController(1);

  private final Intake intake = Intake.getInstance();
  private final Tank tank = Tank.getInstance();
  private final Shooter shooter = Shooter.getInstance();
  private final LED led = LED.getInstance();
  // private final LEDTest ledTest = LEDTest.getInstance();

  private SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    NamedCommands.registerCommand("runIntake", intake.runIntakeCommand());
    NamedCommands.registerCommand("stopIntake", intake.stopIntakeRollerCommand());
    NamedCommands.registerCommand("intakeUp", intake.intakeUpCommand());
    NamedCommands.registerCommand("autoShoot", shooter.autoAimShootCommand());
    NamedCommands.registerCommand("hubShoot", shooter.shootCommandHub());
    NamedCommands.registerCommand("lobShoot", shooter.shootLobCommand());
    NamedCommands.registerCommand("zeroIntake", intake.resetIntakeCommand());
    NamedCommands.registerCommand("wait3Seconds", shooter.wait3SecondsCommands());
    NamedCommands.registerCommand("stopShooter", shooter.stopShooterCommand());

    configureBindings();
    configureSelector();
  }

  private void configureBindings() {
    // RobotConfig config;
    // try{
    //   config = RobotConfig.fromGUISettings();
    // }catch (Exception exception){
    //   exception.printStackTrace();
    // }
    // driveStick.leftBumper()
    //   .onTrue(shooter.autoAimShootCommand())
    //   .onFalse(shooter.stopShooterCommand());
    // driveStick.rightTrigger(0.25)
    //   .onTrue(shooter.decideWhatShoot())
    //   .onFalse(shooter.stopShooterCommand());
      
    driveStick.rightTrigger(0.25)
      .onTrue(shooter.autoShootCommand())
      .onFalse(shooter.stopShooterCommand());

    driveStick.leftTrigger(0.25)
      .onTrue(shooter.shootLobCommand())
      .onFalse(shooter.stopShooterCommand());

    driveStick.leftBumper()
      .onTrue(tank.turnToAngleCommand())
      .onFalse(tank.stopTurnToAngleCommand());

    driveStick.b() //Deploys intake and runs intake, runs conveyor when pressed stows when not pressed
      .onTrue(intake.deployIntakeCommand())
      .onFalse(intake.stowIntakeCommand());
        
    driveStick.rightBumper()
      .onTrue(intake.spinRollerCommand())
      .onFalse(intake.stopIntakeRollerCommand());

    driveStick.a()
      .onTrue(intake.spinRollerCommandMore())
      .onFalse(intake.stopIntakeRollerCommand());
    
    driveStick.y()
      .onTrue(shooter.startIndexerCommand())
      .onFalse(shooter.stopIndexerCommand());

    driveStick.povRight() //Deploys intake and outtakes intake backward, runs conveyor and stows when not pressed
      .onTrue(intake.regurgitIntakeCommand())
      .onFalse(intake.stowIntakeCommand());

    driveStick.povUp()
      .onTrue(shooter.backwardShootCommand())
      .onFalse(shooter.stopShooterCommand());

    driveStick.x() //Stows intake then zero it there
      .onTrue(intake.resetIntakeCommand());

    // driveStick.a()
    //   .onTrue(shooter.shootLobCommand())
    //   .onFalse(shooter.stopShooterCommand());
    
    // driveStick.leftBumper()
    //   .onTrue(Commands.runOnce(() -> tank.setSlowActive(true)))
    //   .onFalse(Commands.runOnce(() -> tank.setSlowActive(false)));

    // AutoBuilder.configure(
    //   () -> tank.getRobotPose(),
    //   (Pose2d pos) -> tank.resetPos(pos),
    //   () -> tank.getRobotChassisSpeeds(), 
    //   (speeds, feedforwards) -> tank.driveRobotRelative(speeds), 
    //   new PPLTVController(0.02), 
    //   tank.getRobotConfig(), 
    //   () -> {
    //     if(DriverStation.getAlliance().isPresent())
    //       return DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    //     return false;
    //   }, 
    //   tank
    // );
    
  }
    
  public static void setRumble(double value){
    driveStick.setRumble(RumbleType.kBothRumble, value);
  }
  
  private void configureSelector(){
    SmartDashboard.putData("Auto/Selector", autoChooser);

    autoChooser.setDefaultOption("No Auto", Commands.print("No Auto"));

    autoChooser.addOption("goToMiddle", Autos.goToMiddle());
    autoChooser.addOption("hubPreload", Autos.hubPreload());
    autoChooser.addOption("awayHubPreload", Autos.awayHubPreload());
    
    // autoChooser = AutoBuilder.buildAutoChooser("No auto");
    // SmartDashboard.putData("Auto/Selector", autoChooser);
  }
  
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}

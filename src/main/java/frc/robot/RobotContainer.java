// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.Choreo;
import choreo.auto.AutoFactory;
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

  private final static CommandXboxController driveStick = new CommandXboxController(0);
  // private final CommandXboxController mechStick = new CommandXboxController(1);

  private final Intake intake = Intake.getInstance();
  private final Tank tank = Tank.getInstance();
  private final Shooter shooter = Shooter.getInstance();
  private final LED led = LED.getInstance();
  // private final LEDTest ledTest = LEDTest.getInstance();

  private SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    configureBindings();
    configureSelector();
  }

  private void configureBindings() {
    
    // driveStick.leftBumper()
    //   .onTrue(shooter.autoAimShootCommand())
    //   .onFalse(shooter.stopShooterCommand());
    driveStick.y()
      .onTrue(shooter.decideWhatShoot())
      .onFalse(shooter.stopShooterCommand());
      
    driveStick.leftTrigger(0.25)
      .onTrue(shooter.shootLobCommand())
      .onFalse(shooter.stopShooterCommand());

    // driveStick.rightBumper()
    //   .onTrue(tank.turnToHubCommand());

    driveStick.rightTrigger(0.25) //Deploys intake and runs intake, runs conveyor when pressed stows when not pressed
      .onTrue(intake.deployIntakeCommand())
      .onFalse(intake.stowIntakeCommand());
        
    driveStick.b()
      .onTrue(intake.spinRollerCommand())
      .onFalse(intake.stopIntakeRollerCommand());

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
    
    driveStick.leftBumper()
      .onTrue(Commands.runOnce(() -> tank.setSlowActive(true)))
      .onFalse(Commands.runOnce(() -> tank.setSlowActive(false)));
  
    tank.setDefaultCommand(
      Commands.run(() -> tank.drive(-driveStick.getLeftY(), driveStick.getRightX()), tank)
    );
  }
  // public void configureBindings(){
  //   driveStick.a()
  //   .onTrue(Commands.runOnce(() -> ledTest.setLEDAtAngle(), ledTest));
  //   driveStick.b()
  //   .onTrue(Commands.runOnce(() -> ledTest.setLEDTankDisable(), ledTest));
  //   driveStick.rightTrigger(.25)
  //   .onTrue(Commands.runOnce(() -> ledTest.setLEDDisable(), ledTest));
  //   driveStick.y()
  //   .onTrue(Commands.runOnce(() -> ledTest.setLEDAutoAiming(), ledTest));
  //   driveStick.x()
  //   .onTrue(Commands.runOnce(() -> ledTest.setLEDShooting(), ledTest));
  //   driveStick.leftTrigger(.25)
  //   .onTrue(Commands.runOnce(() -> ledTest.setLEDIntaking(), ledTest));
  // }

  public static void setRumble(double value){
    driveStick.setRumble(RumbleType.kBothRumble, value);
  }

  private void configureSelector(){
    SmartDashboard.putData("Auto/Selector", autoChooser);
    autoChooser.setDefaultOption("No auto", Commands.print("No auto"));

    autoChooser.addOption("midPreload", Autos.midPreload());
    autoChooser.addOption("leftPreload", Autos.leftPreload());
    autoChooser.addOption("rightPreload", Autos.rightPreload());
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}

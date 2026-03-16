// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tank;

public class RobotContainer {

  private final CommandXboxController driveStick = new CommandXboxController(0);
  private final CommandXboxController mechStick = new CommandXboxController(0);

  private final Intake intake = Intake.getInstance();
  private final Tank tank = Tank.getInstance();
  private final Shooter shooter = Shooter.getInstance();

  private SendableChooser <Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {

    configureBindings();
    configureSelector();
  }

  private void configureBindings() {
    
    this.mechStick.a()
      .whileTrue(Commands.sequence(
        tank.turnToHubCommand(),
        shooter.autoAimShootCommand()
      )
    );

    this.mechStick.leftBumper()
      .onTrue(shooter.shootCommandHub());
      
    this.mechStick.leftTrigger(0.25)
      .onTrue(shooter.shootCommandAwayHub());

    this.mechStick.rightTrigger(0.35) //Deploys intake and runs intake, runs conveyor when pressed stows when not pressed
      .onTrue(intake.deployIntakeCommand())
      .onFalse(intake.stowIntakeCommand());
        
    this.mechStick.b() //Deploys intake and outtakes intake backward, runs conveyor and stows when not pressed
      .onTrue(intake.regurgitateIntakeCommand())
      .onFalse(intake.stowIntakeCommand());


    tank.setDefaultCommand(
      Commands.run(() -> tank.drive(-driveStick.getLeftY(), driveStick.getRightX()), tank)
    );
  }

  private void configureSelector(){
    autoChooser.setDefaultOption("No auto", Commands.print("No auto"));

    autoChooser.addOption("midPreload", Autos.midPreload());
    autoChooser.addOption("leftPreload", Autos.leftPreload());
    autoChooser.addOption("rightPreload", Autos.rightPreload());
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
  private final LED led = LED.getInstance();

  private final AutoFactory autoFactory;
  private final Autos autos;
  private SendableChooser <Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    autoFactory = tank.createAutoFactory();

    configureBindings();
  }

  private void configureBindings() {
    
    this.mechStick.b()
      .whileTrue(Commands.sequence(
        tank.turnCommand(),
        shooter.autoAimshootCommand()
      ));

    this.mechStick.rightTrigger() //Deploys intake and runs intake, runs conveyor when pressed stows when not pressed
      .whileTrue(intake.deployIntakeCommand())
      .whileFalse(intake.stowIntakeCommand());
        
    this.mechStick.a() //Deploys intake and outtakes intake backward, runs conveyor and stows when not pressed
      .whileTrue(intake.regurgitateIntakeCommand())
      .whileFalse(intake.stowIntakeCommand());


    tank.setDefaultCommand(
      Commands.run(() -> tank.drive(-driveStick.getLeftY(), driveStick.getRightX()), tank)
    );
  }

  private void configureSelector(){
    autoChooser.addOption("rightPile", autos.rightPile());
    autoChooser.addOption("leftPile", autos.leftPile());
    autoChooser.addOption("midPile", autos.midPile());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

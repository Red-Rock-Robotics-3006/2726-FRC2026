// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tank;

public class RobotContainer {

  private final CommandXboxController driveStick = new CommandXboxController(0);
  private final CommandXboxController mechStick = new CommandXboxController(0);

  private final Conveyor conveyor = Conveyor.getInstance();
  private final Intake intake = Intake.getInstance();
  private final Tank tank = Tank.getInstance();
  private final Shooter shooter = Shooter.getInstance();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    
    this.mechStick.b()
      .whileTrue(Commands.sequence(
        tank.turnCommand(),
        shooter.shootIndexCommand()
      ));

    this.mechStick.rightTrigger() //Deploys intake and runs intake, runs conveyor when pressed stows when not pressed
      .whileTrue(intake.startIntakingCommand())
      .whileFalse(intake.stopIntakingCommand());
        
    this.mechStick.a() //Deploys intake and outtakes intake backward, runs conveyor and stows when not pressed
      .whileTrue(intake.startOuttakingCommand())
      .whileFalse(intake.stopIntakingCommand());

    this.mechStick.x()  //Runs conveyor backward
      .whileTrue(conveyor.feedConveyorBackwardCommand())
      .whileFalse(conveyor.stopConveyorCommand());

    tank.setDefaultCommand(
      Commands.run(() -> tank.drive(-driveStick.getLeftY(), driveStick.getRightX()), tank)
    );
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

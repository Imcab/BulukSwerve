// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.swervechasis;
import frc.robot.command.driveCommand;

public class RobotContainer {

  private final swervechasis chasis = new swervechasis();

  public CommandXboxController driverjoytick = new CommandXboxController(0);
 
  public RobotContainer() {
   
    driveCommand commandDrive = new driveCommand(chasis,
      ()-> driverjoytick.getRawAxis(XboxController.Axis.kLeftX.value),
      ()-> driverjoytick.getRawAxis(XboxController.Axis.kLeftY.value),
      ()-> driverjoytick.getRawAxis(XboxController.Axis.kRightX.value));


    chasis.setDefaultCommand(commandDrive
    );

    configureBindings(); 

  }

  private void configureBindings() { 
  
  }
  
  public Command getAutonomousCommand() {
    return Commands.print("No autonomo");
    
  }
  
}


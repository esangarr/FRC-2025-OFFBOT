// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.DriveCommands.DriveCommands;
import frc.robot.DriveTrain.Swerve;
import frc.robot.DriveTrain.Swerve.SwervePathConstraints;
import frc.robot.DriveTrain.Vision;
import lib.ForgePlus.NetworkTableUtils.NTPublisher;
import lib.ForgePlus.NetworkTableUtils.NTSendableChooser;

public class RobotContainer {

  private final Swerve chassis;
  private final Vision vision;

  private final CommandXboxController driver = new CommandXboxController(0);

  private PathPlannerAuto lateral;
  private PathPlannerAuto mover;
  private PathPlannerAuto inicio;
  private PathPlannerAuto rotar;

  public NTSendableChooser<Command> autoChooser = new NTSendableChooser<>(NTPublisher.ROBOT, "AutoSelector");


  public RobotContainer() {

    chassis = new Swerve(SwervePathConstraints.kNormal);
    vision = new Vision("Arducam1",     
                        new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0)),//Poner posicion de la camara respecto al robot  
                        chassis::addVisionMeasurement);
                          
    NTPublisher.publish("Joysticks", "Driver1", driver);
  
    lateral = new PathPlannerAuto("lat");
    mover = new PathPlannerAuto("mover");
    inicio = new PathPlannerAuto("inicio");
    rotar = new PathPlannerAuto("llegarRot");


    autoChooser.setDefault("mover", mover).
    add("inicio", inicio).
    add("lat", lateral).add("rot", rotar).
    publish();
  
    configureBindings();

  }

  private void configureBindings() {
    
    chassis.setDefaultCommand(DriveCommands.joystickDrive(chassis, ()-> -driver.getLeftY(), ()-> -driver.getLeftX(), ()-> -driver.getRightX()));

    
    //driver.x().whileTrue(chassis.getPathFinder().toPoseCommand(new Pose2d(0.53, 3.6, Rotation2d.kZero)));
    
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

}
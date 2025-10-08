// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.DriveCommands.DriveCommands;
import frc.robot.DriveTrain.Swerve;
import frc.robot.DriveTrain.Swerve.SwervePathConstraints;
import frc.robot.DriveTrain.Vision;
import frc.robot.MecaCommands.ClimberCommands.ClimberCommands;
import frc.robot.MecaCommands.ElevatorCommands.ElevatorCommands;
import frc.robot.MecaCommands.IntakeCommands.IntakeCommands;
import frc.robot.MecaCommands.OutakeCommands.OutakeCommands;
import frc.robot.Mechanisms.Climber.ClimberSub;
import frc.robot.Mechanisms.Elevator.ElevatorSub;
import frc.robot.Mechanisms.Indexer.IndexerSub;
import frc.robot.Mechanisms.Intake.IntakeSub;
import frc.robot.Mechanisms.Outake.OutakeSub;
import lib.ForgePlus.NetworkTableUtils.NTPublisher;
import lib.ForgePlus.NetworkTableUtils.NTSendableChooser;
import lib.ForgePlus.NetworkTableUtils.NTTunnableNumber;
import lib.ForgePlus.SwerveLib.Utils.Smoothjoystick;

public class RobotContainer {

  private final Swerve chassis;
  private final Vision vision;

  private final IntakeSub intake;
  private final IndexerSub index;
  private ElevatorSub elevator;
  private final OutakeSub outake;
  private final ClimberSub climber;
  private final Debouncer timerOut = new Debouncer(0.45);


  private final Smoothjoystick smooth = new Smoothjoystick(1.1);

  private CommandXboxController driver = new CommandXboxController(0);
  private CommandXboxController operator = new CommandXboxController(1);

  private PathPlannerAuto lateral;
  private PathPlannerAuto mover;
  private PathPlannerAuto inicio;
  private PathPlannerAuto rotar;

  public NTSendableChooser<Command> autoChooser = new NTSendableChooser<>(NTPublisher.ROBOT, "AutoSelector");


  public RobotContainer() {

    chassis = new Swerve(SwervePathConstraints.kNormal);
    vision = new Vision("Arducam_1",     
                        new Transform3d(new Translation3d(0-.2583434, -0.3018282, 0.206375), new Rotation3d(0.491, 0, 0.869)),//Poner posicion de la camara respecto al robot  
                        chassis::addVisionMeasurement);
    
    intake = new IntakeSub();
    elevator = new ElevatorSub();
    index = new IndexerSub();
    outake = new OutakeSub();
    climber = new ClimberSub();

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

    //---------------------------------------------------------------- DRIVER ----------------------------------------------------------------

    chassis.setDefaultCommand(DriveCommands.joystickDrive(
      chassis,
      smooth.filter(()-> -driver.getLeftY() * 0.9),
      smooth.filter(()-> -driver.getLeftX() * 0.9),
      smooth.filter(()-> -driver.getRightX() * 0.8)));
    
    chassis.setDefaultCommand(DriveCommands.joystickDrive(chassis, ()-> driver.getLeftY(), ()-> driver.getLeftX(), ()-> -driver.getRightX()));

    driver.a().whileTrue(chassis.getPathFinder().toPoseCommand(new Pose2d(3.19,4.03, Rotation2d.kZero)));
    driver.y().whileTrue(DriveCommands.resetHeading(chassis));

    driver.povLeft().whileTrue(DriveCommands.moveInX(chassis, 0.6));
    driver.povRight().whileTrue(DriveCommands.moveInX(chassis, -0.6));
    driver.povUp().whileTrue(DriveCommands.moveInY(chassis, 0.6));
    driver.povDown().whileTrue(DriveCommands.moveInY(chassis,-0.6));

    //---------------------------------------------------------------- DRIVER ----------------------------------------------------------------

    //---------------------------------------------------------------- OPERATOR ----------------------------------------------------------------

    NTTunnableNumber num = new NTTunnableNumber("Tunnables/kS", 0);
  
    operator.leftStick().whileTrue(IntakeCommands.runIntakeManual(intake, ()-> operator.getLeftY() * 0.6));
    operator.rightStick().whileTrue(ElevatorCommands.runManual(elevator, ()-> operator.getRightY()*0.15));

    //operator.b().whileTrue(IntakeCommands.setAngleUp(intake, 30));
    //operator.x().whileTrue(IntakeCommands.setAngleDown(intake,210));
    operator.y().whileTrue(IntakeCommands.outPiece(intake, index, 0.9, 30, 0.5 ));

    operator.x().whileTrue(ElevatorCommands.setPos(elevator, 10));
    operator.a().whileTrue(ElevatorCommands.setPos(elevator, 38));

    operator.y().whileTrue(IntakeCommands.clearPiece(intake, index, 0.7, 0.5, 0.5, timerOut));

    operator.rightBumper().whileTrue(OutakeCommands.setAngleUp(outake, 130));
    operator.leftBumper().whileTrue(OutakeCommands.setAngleDown(outake, 0));

    operator.leftTrigger().whileTrue(OutakeCommands.outWheels(outake, 0.6)); //Tragar
    operator.rightTrigger().whileTrue(OutakeCommands.outWheels(outake, -0.2)); //Disparar

    operator.povDown().whileTrue(OutakeCommands.resetEncoder(outake));

    
    

    //---------------------------------------------------------------- OPERATOR ----------------------------------------------------------------


  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

}
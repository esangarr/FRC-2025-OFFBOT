// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.AutoCommands.AutoTakeCoral;
import frc.robot.AutoCommands.ElevatorAuto;
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
import frc.robot.Mechanisms.Elevator.ElevatorSub.RequestType;
import frc.robot.Mechanisms.Indexer.IndexerSub;
import frc.robot.Mechanisms.Intake.IntakeSub;
import frc.robot.Mechanisms.Outake.OutakeSub;
import frc.robot.Mechanisms.Outake.OutakeSub.OutakeRequestType;
import lib.ForgePlus.NetworkTableUtils.NTPublisher;
import lib.ForgePlus.NetworkTableUtils.NTSendableChooser;
import lib.ForgePlus.SwerveLib.Utils.Smoothjoystick;

public class RobotContainer {

  private final Swerve chassis;
  private final Vision vision;

  private final IntakeSub intake;
  private final IndexerSub index;
  private final ElevatorSub elevator;
  private final OutakeSub outake;
  private final ClimberSub climber;
  private final Debouncer timerOut = new Debouncer(0.42);


  private final Smoothjoystick smooth = new Smoothjoystick(1.1);

  private CommandXboxController driver = new CommandXboxController(0);
  private CommandXboxController operator = new CommandXboxController(1);

  private PathPlannerAuto mover;
  private PathPlannerAuto giro;
  private PathPlannerAuto L4;


  public SendableChooser<Command> chooser = new SendableChooser<>();


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

    NamedCommands.registerCommand("TakeCoral", ElevatorCommands.takeCoral(elevator, outake, elevator.metersToRot(85), outake.DegreesToRotations(2)).withTimeout(0.8));
    NamedCommands.registerCommand("UpL4", ElevatorCommands.scoreCoral2(elevator, outake, elevator.metersToRot(187), outake.DegreesToRotations(120)).withTimeout(2));
    NamedCommands.registerCommand("Retract", new ElevatorAuto(elevator, outake, elevator.metersToRot(80), outake.DegreesToRotations(2.5)));
    NamedCommands.registerCommand("AutoDunk", OutakeCommands.shootDunk(outake, 0.4).withTimeout(0.05));
    NamedCommands.registerCommand("Home", ElevatorCommands.setPosDown(elevator, outake, elevator.metersToRot(80), outake.DegreesToRotations(2)).withTimeout(2));


    mover = new PathPlannerAuto("Auto1");
    giro = new PathPlannerAuto("AutoGiro");
    L4 = new PathPlannerAuto("ScoreL4");

    chooser.addOption("AutoGiro", giro);
    chooser.addOption("L4", L4);
    chooser.setDefaultOption("Mover", mover);

    SmartDashboard.putData("AutoSelector", chooser);
  
    configureBindings();

  }

  private void configureBindings() {

    //---------------------------------------------------------------- DRIVER ----------------------------------------------------------------
    chassis.setDefaultCommand(DriveCommands.joystickDrive(
      chassis,
      smooth.filter(()-> -driver.getLeftY() * 0.55),
      smooth.filter(()-> -driver.getLeftX() * 0.55),
      smooth.filter(()-> -driver.getRightX() * 0.4)));

   
    driver.rightBumper().whileTrue(ClimberCommands.angleClimber(climber, 1));
    driver.leftBumper().whileTrue(ClimberCommands.angleClimber(climber, -1));

    driver.leftTrigger().whileTrue(ClimberCommands.climberWheels(climber, -1));

    driver.x().whileTrue(chassis.getPathFinder().toPoseCommand(new Pose2d(3.19,4.03, Rotation2d.kZero)));
    driver.b().whileTrue(DriveCommands.resetHeading(chassis));

    driver.povLeft().whileTrue(DriveCommands.moveInX(chassis, 0.35));
    driver.povRight().whileTrue(DriveCommands.moveInX(chassis, -0.35));
    driver.povUp().whileTrue(DriveCommands.moveInY(chassis, 0.35));
    driver.povDown().whileTrue(DriveCommands.moveInY(chassis,-0.35));

    driver.y().toggleOnTrue(IntakeCommands.setAngleUp(intake, 35)); // Cambiar a Driver
    driver.a().whileTrue(IntakeCommands.setAngleDown(intake, 240)); // Cambiar a Driver

    //---------------------------------------------------------------- DRIVER ----------------------------------------------------------------

    //---------------------------------------------------------------- OPERATOR ----------------------------------------------------------------
    outake.setDefaultCommand(OutakeCommands.armManual(outake, ()-> -operator.getRightY()*0.2));
    elevator.setDefaultCommand(ElevatorCommands.runManual(elevator, ()-> operator.getLeftY()*0.2));
    
    
    operator.a().toggleOnTrue(ElevatorCommands.scoreCoral2(elevator, outake,  elevator.metersToRot(88), outake.DegreesToRotations(80)));
    operator.x().toggleOnTrue(ElevatorCommands.scoreCoral2(elevator, outake,  elevator.metersToRot(72), outake.DegreesToRotations(125)));

    operator.b().toggleOnTrue(ElevatorCommands.scoreCoral2(elevator, outake, elevator.metersToRot(117), outake.DegreesToRotations(128)));
    operator.y().toggleOnTrue(ElevatorCommands.scoreCoral2(elevator, outake, elevator.metersToRot(187), outake.DegreesToRotations(119)));

    operator.povDown().whileTrue(ElevatorCommands.setPosDown(elevator, outake, elevator.metersToRot(80), outake.DegreesToRotations(2.5)));
    operator.leftStick().whileTrue(ElevatorCommands.takeCoral(elevator, outake, elevator.metersToRot(85), outake.DegreesToRotations(2.5)));


    operator.povLeft().toggleOnTrue(ElevatorCommands.GetAlgae(elevator, outake, elevator.metersToRot(93), outake.DegreesToRotations(90), -0.32, RequestType.kUP, OutakeRequestType.kUp));
    operator.povRight().toggleOnTrue(ElevatorCommands.GetAlgae(elevator, outake, elevator.metersToRot(140), outake.DegreesToRotations(82), -0.32,RequestType.kUP, OutakeRequestType.kUp));

    operator.rightStick().toggleOnTrue(ElevatorCommands.GetAlgae(elevator, outake, elevator.metersToRot(80), outake.DegreesToRotations(135), -0.32,RequestType.kDown, OutakeRequestType.KAlgae));
    operator.povUp().toggleOnTrue(ElevatorCommands.GetAlgae(elevator, outake, elevator.metersToRot(187), outake.DegreesToRotations(140), -0.32, RequestType.kUP, OutakeRequestType.KAlgae));


    operator.leftBumper().whileTrue(IntakeCommands.outPiece(intake, index, 0.9, 35, 0.5 ));
    operator.rightBumper().whileTrue(IntakeCommands.clearPiece(intake, index, elevator, outake, 0.9, 0.5, 0.5 , timerOut, 35, 200));

    operator.start().whileTrue(OutakeCommands.resetArmEncoder(outake));

    operator.leftTrigger().whileTrue(OutakeCommands.moveWheels(outake, 1));
    operator.rightTrigger().whileTrue(OutakeCommands.shootDunk(outake, 0.4)); 


    //---------------------------------------------------------------- OPERATOR ----------------------------------------------------------------


  }

  public Command getAutonomousCommand() {
    return chooser.getSelected();
  }

}
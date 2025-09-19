package frc.robot.MecaCommands.IntakeCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
//import frc.robot.Mechanisms.Indexer.IndexerSub;
import frc.robot.Mechanisms.Intake.IntakeSub;

public class IntakeCommands {

    public static Command setAngle(IntakeSub intake, double angle){

        return Commands.run(()-> {
            intake.setPosition(angle);
        },
        intake);
    }
/* 
    public static Command eatPiece(
        IntakeSub intake,
        IndexerSub index,
        double eatAngle,
        double retractAngle,
        double wheelsIntake,
        double wheelsIndex ){

            return Commands.run(()->{
                intake.setPosition(eatAngle);

              if (index.hasPiece() == false){
                    intake.runWheelsIntake(wheelsIntake);
                  index.runWheels(wheelsIndex);
                }else{
                    intake.stopWheelsIntake();
                    index.stop();
                    intake.setPosition(retractAngle);
                }

            }, intake, index).finallyDo(()->{intake.stopAll(); index.stop();});
    }

    public static Command eatPieceNoBeam(
        IntakeSub intake,
        IndexerSub index,
        double angle,
        double wheelsIntake,
        double wheelsIndex){

            return Commands.run(()->{
                intake.setPosition(angle);
                intake.runWheelsIntake(wheelsIntake);
                index.runWheels(wheelsIndex);

          },intake, index).finallyDo(()-> {

            intake.stopAll();
            index.stop();});
    }
*/

    public static Command runIntakeWheels(IntakeSub intake, double speed){
        return Commands.run(()-> {
            intake.runWheelsIntake(speed);
        },intake).finallyDo(()->{intake.stopWheelsIntake();});
    }

    public static Command runIntakeManual(IntakeSub intake, DoubleSupplier joystick){
        return Commands.run(()->{
            intake.runIntake(joystick.getAsDouble());
        }, intake).finallyDo(()->{intake.stopAng();});
    }




}

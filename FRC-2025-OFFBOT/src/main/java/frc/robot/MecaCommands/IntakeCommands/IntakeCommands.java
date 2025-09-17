package frc.robot.MecaCommands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Mechanisms.Indexer.IndexerSub;
import frc.robot.Mechanisms.Intake.IntakeSub;

public class IntakeCommands {

    public static Command setAngle(IntakeSub intake, double angle){

        return Commands.run(()-> {
            intake.setPosition(angle);
        },
        intake);
    }

    public static Command eatPiece(IntakeSub intake, IndexerSub index, double eatAngle, double retractAngle, double wheelsIntake, double wheelsIndex){
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

    public static Command moveAngleSpeed(IntakeSub intake, double speed){

        return Commands.run(()-> {
            intake.runIntake(speed);
        },
        intake).finallyDo(()->{
            intake.stopAng();
        });
    }




}

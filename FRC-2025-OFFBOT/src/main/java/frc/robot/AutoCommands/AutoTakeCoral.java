package frc.robot.AutoCommands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Mechanisms.Elevator.ElevatorSub;
import frc.robot.Mechanisms.Elevator.ElevatorSub.RequestType;
import frc.robot.Mechanisms.Indexer.IndexerSub;
import frc.robot.Mechanisms.Outake.OutakeSub;
import frc.robot.Mechanisms.Outake.OutakeSub.OutakeRequestType;

public class AutoTakeCoral extends Command{

    private ElevatorSub elevator;
    private OutakeSub outake;
    private IndexerSub index;

    private Debouncer timer;

    
    public AutoTakeCoral (ElevatorSub  elevator, OutakeSub outake, IndexerSub index ){

        this.elevator = elevator;
        this.index = index;
        this.outake = outake;

        timer = new Debouncer(0.4);

    }

    @Override
    public void initialize(){
        
    }

    @Override
    public void execute(){

        elevator.setPosition(-elevator.metersToRot(63), RequestType.kDown);
        outake.runWheelsOutake(-0.6);
        outake.setPosition(outake.DegreesToRotations(2), OutakeRequestType.KDown);

        if (index.noPiece()){
            outake.runWheelsOutake(0);     
            elevator.setPosition(-elevator.metersToRot(85), RequestType.kDown);
            outake.setPosition(outake.DegreesToRotations(2), OutakeRequestType.KDown);
        }
    


    }


    @Override
    public void end(boolean interrupted) {
       
    }

    @Override
    public boolean isFinished(){ 
        return timer.calculate(index.noPiece());  
        
    }
}

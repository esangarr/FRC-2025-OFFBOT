package frc.robot.Mechanisms.Outake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;


public class Wheels extends SubsystemBase {
    /*      TODO:
      Add JM's code. After this, we can make the constants
    */    
    private TalonFX Wheel;
    private TalonFXConfiguration wheelMotor1Config;//TODO CONFIG

 public Wheels() {
      TalonFX Wheel = new TalonFX(OutConstants.Wheel_ID);

 
// final TalonFXSimState wheelMotor1_Sim = Wheel.getSimState();

 //final DutyCycleOut m_talonFXOut = new DutyCycleOut(0);
 Wheel.optimizeBusUtilization();
 final TalonFXConfiguration m_talonFXConfig = new TalonFXConfiguration();
 Wheel.getConfigurator().apply(m_talonFXConfig);
 }
  public void runWheels(double speed) {
   // Implementation for running wheels
   Wheel.setControl(new VelocityVoltage(speed));
   }


  public void stopWheels() {
   // Implementation for stopping wheels
   Wheel.setControl(new NeutralOut());
   }
}

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OperatorConstants.PneumaticsConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;

public class RobotGrabberSubsystem extends SubsystemBase {
    private final DoubleSolenoid solenoid1;
    // private final Compressor phCompressor;
     
    
    
    public RobotGrabberSubsystem() {
        solenoid1 = new DoubleSolenoid(14, PneumaticsModuleType.REVPH, 
        PneumaticsConstants.forwardChannel, PneumaticsConstants.reverseChannel);

        solenoid1.set(Value.kReverse);
        
  
        // phCompressor = new Compressor(0, PneumaticsModuleType.REVPH);


    }

    @Override
    public void periodic() {
       
        // Debug Info
        SmartDashboard.putString("Solenoid Value", solenoid1.get().toString());
        // SmartDashboard.putNumber("PSI", phCompressor.getPressure());
    }



    //toggles the grabber
    public void toggleGrabber() {
        solenoid1.toggle();
    }

    //Maybe will close the grabber
    public void openGrabber() {
        solenoid1.set(Value.kForward);
    }
    //Maybe will close the grabber
    public void closeGrabber() {
        solenoid1.set(Value.kReverse);
    }

    
}
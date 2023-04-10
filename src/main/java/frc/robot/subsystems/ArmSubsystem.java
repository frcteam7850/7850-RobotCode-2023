package frc.robot.subsystems;
import java.beans.VetoableChangeSupport;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OperatorConstants.ArmConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;

// import frc.robot.commands.ArmUpCmd;


public class ArmSubsystem extends SubsystemBase{
    //Motors
    private final CANSparkMax armMotor1;
    private final CANSparkMax armMotor2;
    private final CANSparkMax armMotor3;

    //Motor Encoders
    private final RelativeEncoder armMotor1Encoder;
    private final RelativeEncoder armMotor2Encoder;
    private final RelativeEncoder armMotor3Encoder;

    private double horizSpeed = 1;

    private boolean lim = true;

    private DigitalInput limitSwitch = new DigitalInput(0);
    

    //creating the command
    // public void setDefaultCommand(ArmUpCmd armOpenCmd) {
    // }

    public ArmSubsystem(){

        //Motor id's
        armMotor1 = new CANSparkMax(44, MotorType.kBrushless);
        armMotor2 = new CANSparkMax(45, MotorType.kBrushless);
        armMotor3 = new CANSparkMax(46, MotorType.kBrushless);

        //Encoders
        armMotor1Encoder = armMotor1.getEncoder();
        armMotor2Encoder = armMotor2.getEncoder();
        armMotor3Encoder = armMotor3.getEncoder();
    }

    @Override
    public void periodic() {
       
        // Debug Info
        SmartDashboard.putNumber("ArmMotor1 Pos", armMotor1Encoder.getPosition());
        SmartDashboard.putNumber("ArmMotor2 Pos", armMotor2Encoder.getPosition());
        SmartDashboard.putNumber("ArmMotor3 Pos", armMotor3Encoder.getPosition());

        SmartDashboard.putBoolean("Limit Switch", !getLimitSwitch());

        //  if (!getLimitSwitch()){
        //     while (!getLimitSwitch()){
        //     setHorizMotors(-0.01);
        //     System.out.println("Limit Switch Hit");
        //  }
        //  } else if (getHorizEncoder() < -279 && armMotor3Encoder.getVelocity() < 0){
        //      setHorizMotors(0.01);
        //      System.out.println("Encoder limit reached");
        //  }


    }

    // extends/retracts arm
     public void testExtension() {
         armMotor3.set(-horizSpeed);
     }

     public void testRetraction() {
         armMotor3.set(horizSpeed);
     }

    public void stopVertMotors() {

        armMotor1.set(0);
        armMotor2.set(0);

    }

     public void stopHorizMotors() {

        armMotor3.set(0);

     }

    public void setVertMotors(double speed){
        armMotor1.set(-speed);
        armMotor2.set(speed);
    }

     public void setHorizMotors(double speed){
         armMotor3.set(speed);
     }

    public double getVertEncoder() {
        return armMotor1Encoder.getPosition();
    }
     public double getHorizEncoder() {
         return armMotor3Encoder.getPosition();
    }

    public void resetVertEncoders() {
        armMotor1Encoder.setPosition(0);
        armMotor2Encoder.setPosition(0);
    }

     public void resetHorizEncoder() {
         armMotor3Encoder.setPosition(0);
    }

    public boolean getLimitSwitch(){
        return limitSwitch.get();

    }  

 
    
    public void encoderCalibrator(){
    while (lim == true){
    
    if (getLimitSwitch()) {

      setHorizMotors(0.75);

    } else if (!getLimitSwitch()){

            stopHorizMotors();
            resetHorizEncoder();
            lim = false;
          
       }
      }
    }
    }



  






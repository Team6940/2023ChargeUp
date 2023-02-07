package frc.robot.subsystems;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.*;


public class Claw extends SubsystemBase{
    int CLAW_CHANNEL = 0;
    Spark spark = null;
    PWMMotorController motor =  MotorMap.ClawMotor;

    public void initDefaultCommand(){

    }

    public void open(){
        motor.set(1);
    }

    public void close(){
        motor.set(-1);
    }
    public void stop(){
        motor.set(0);
    }
}

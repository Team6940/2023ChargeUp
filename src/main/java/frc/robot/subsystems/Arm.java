package frc.robot.subsystems;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

public class Arm extends SubsystemBase{
    WPI_TalonFX Motor1 = new WPI_TalonFX(1, "l");//等改
    WPI_TalonFX Motor2 = new WPI_TalonFX(1, "l");

    public Arm (){
        
    }
    @Override
    public void periodic() {

    }
}

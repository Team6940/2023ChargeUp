package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
//import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
    public static Elevator instance = null;
    public static boolean m_ElevatorEnabled=false;
    private TalonFX m_ElevatorMotor;

    //private PIDController m_ElevatorMotorInPIDCOntroller=new PIDController(ElevatorConstants.ElevatorMotorInkP,ElevatorConstants.ElevatorMotorInkI,ElevatorConstants.ElevatorMotorInkD);
    //private PIDController m_ElevatorMotorOutPIDCOntroller=new PIDController(ElevatorConstants.ElevatorMotorOutkP,ElevatorConstants.ElevatorMotorOutkI,ElevatorConstants.ElevatorMotorOutkD);

    public static Elevator GetInstance()
    {
        if(instance==null)
        {
            instance=new Elevator();
            return instance;
        }
        else
            return instance;
    }
    public Elevator(){
        m_ElevatorMotor=new TalonFX(ElevatorConstants.ElevatorMotorDeviceNumber);
        
        m_ElevatorMotor.setInverted(true);   // 上机调试是否反向

        m_ElevatorMotor.config_kP(0, ElevatorConstants.ElevatorMotorInkP);
        m_ElevatorMotor.config_kI(0, ElevatorConstants.ElevatorMotorInkI);
        m_ElevatorMotor.config_kD(0, ElevatorConstants.ElevatorMotorInkD);
        m_ElevatorMotor.configPeakOutputForward(1);//输出输入最大值，可放Constants也可直接在这里改
        m_ElevatorMotor.configPeakOutputReverse(-1);

        m_ElevatorMotor.configMotionAcceleration(11111);//待调试
        m_ElevatorMotor.configMotionCruiseVelocity(11111);
    }
    
    public void outputTelemetry(){
        SmartDashboard.putString("Debug/Elevator/WantedElevatorState", getWantedElevatorState().name());
    }

    public enum ElevatorState {
        PUSH, PULL, INIT
    }
    
    private ElevatorState wantedElevatorState = ElevatorState.INIT;
    
      // this a extern func for other command call.
    public synchronized void setWantedElevatorState(ElevatorState ElevatorState) {
        wantedElevatorState = ElevatorState;
    }
        
    public synchronized ElevatorState getWantedElevatorState() {
        return wantedElevatorState ;
    }
    
    private void setElevatorMotor(double position) {
        m_ElevatorMotor.set(ControlMode.MotionMagic, position);
    }
    
    private void setElevatorState(ElevatorState ElevatorState) {
        switch (ElevatorState) {
            case PUSH:
                setElevatorMotor(1000001);//待调整
                break;
            case PULL:
                setElevatorMotor(1000000);
                break;
            case INIT:
                setElevatorMotor(0);
                break;
        }
    }

    private int fuck = 0;//继承于李嘉安
    public void autoturnElevator()
    {
        if (fuck % 2 == 0) {//学习来的
        setWantedElevatorState(ElevatorState.PUSH);
        }else{
        setWantedElevatorState(ElevatorState.PULL);
        }
        fuck++;
    }
    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        setElevatorState(wantedElevatorState); 
        outputTelemetry();     
    }
}

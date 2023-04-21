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
    }
    
    public void outputTelemetry(){
        SmartDashboard.putString("Debug/Elevator/WantedElevatorState", getWantedElevatorState().name());
    }

    public enum ElevatorState { //enum是枚举类
        INIT,Stretch_Mid,Stretch_Lower,Stretch_Upper
    }
    
    private ElevatorState _wantedElevatorState = ElevatorState.INIT;
    
      // this a extern func for other command call.
    public synchronized void setWantedElevatorState(ElevatorState _ElevatorState) {
        _wantedElevatorState = _ElevatorState;
    }
        
    public synchronized ElevatorState getWantedElevatorState() {
        return _wantedElevatorState ;
    }
    
    private void setElevatorMotor(double position) {
        m_ElevatorMotor.set(ControlMode.MotionMagic, position);
    }
    
    private void setElevatorState(ElevatorState ElevatorState) {
        switch (ElevatorState) {
            case Stretch_Lower:
                setElevatorMotor(1000001);//待调整
                break;
            case Stretch_Mid:
                setElevatorMotor(1000000);
                break;
            case Stretch_Upper:
                setElevatorMotor(10000000);
                break;
            case INIT:
                setElevatorMotor(100000);
                break;
        }
    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        setElevatorState(_wantedElevatorState); 
        outputTelemetry();     
    }
}

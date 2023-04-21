package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
//import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ArmNewConstants;


public class ArmNew extends SubsystemBase{
    public static ArmNew instance;
    public static boolean m_ElevatorEnabled=false;
    private TalonFX m_ArmMotor;

    public static ArmNew GetInstance()
    {
        if(instance==null)
        {
            instance=new ArmNew();
            return instance;
        }
        else
            return instance;
    }

    public ArmNew(){
        m_ArmMotor=new TalonFX(ArmNewConstants.ArmMotorDeviceNumber);
        
        m_ArmMotor.setInverted(true);   // 上机调试是否反向
        m_ArmMotor.setNeutralMode(NeutralMode.Brake);
    }
    public enum ArmState{
        Score, Grab, Init
    }

    public void outputTelemetry(){
        SmartDashboard.putString("Debug/ArmNew/WantedArmState", getWantedArmState().name());
    }
    private ArmState _wantedArmState = ArmState.Init;

    public synchronized void setWantedArmState(ArmState _ArmState) {
        _wantedArmState = _ArmState;
    }
        
    public synchronized ArmState getWantedArmState() {
        return _wantedArmState ;
    }
    
    private void setArmMotor(double position) {
        m_ArmMotor.set(ControlMode.MotionMagic, position);
    }
    
    private void setArmState(ArmState ArmState) {
        switch (ArmState) {
            case Score:
                setArmMotor(10000);
                break;
            case Grab:
                setArmMotor(10000);
                break;
            case Init:
                setArmMotor(10000);
                break;
        }
    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        setArmState(_wantedArmState); 
        outputTelemetry();     
    }
}

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.FeedbackDevice;//用不到这些
//import com.ctre.phoenix.motorcontrol.TalonFXControlMode;//这行和下一行只要import一个就可以了
import com.ctre.phoenix.motorcontrol.can.TalonFX;

//import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class Intake extends SubsystemBase {

    public static Intake instance = null;
    public static boolean m_IntakeEnabled=false;

    private TalonFX m_IntakeMotorLeft;
    private TalonFX m_IntakeMotorRight;

    private  Solenoid m_IntakeSolenoid;

    
    public static Intake GetInstance()
    {
        if(instance==null)
        {
            instance=new Intake();
            return instance;
        }
        else
            return instance;
    }
    public Intake(){
        m_IntakeMotorLeft=new TalonFX(IntakeConstants.IntakeMotorDeviceNumberLeft);
        m_IntakeMotorRight=new TalonFX(IntakeConstants.IntakeMotorDeviceNumberRight);

        m_IntakeMotorLeft.setInverted(true);   // 上机调试是否反向
        m_IntakeMotorRight.setInverted(false);   // 上机调试是否反向

        m_IntakeSolenoid =new Solenoid(PneumaticsModuleType.CTREPCM,IntakeConstants.IntakeSolenoidPort);

    }

    public void OutPutTelemetry(){
        SmartDashboard.putNumber("Debug/Intake/CMotorOutput: ", m_IntakeMotorLeft.getMotorOutputPercent());//在调试板上显示电机输出百分比
        SmartDashboard.putNumber("Debug/Intake/CMotorOutput: ", m_IntakeMotorRight.getMotorOutputPercent());//在调试板上显示电机输出百分比
        SmartDashboard.putString("Debug/Intake/WantedIntake State", getWantedIntakeState().name());//在调试版上输出intake的目标状态
        SmartDashboard.putString("Debug/Intake/WantedIntakeSolenoid State", getWantedIntakeSolenoidState().name());//在调试版上输出气动杆的目标状态

    }

    public enum IntakeState {
        INTAKE, EJECT,  OFF
    }
    private IntakeState _wantedIntakeState = IntakeState.OFF;

    public enum IntakeSolenoidState{
        OPEN,  CLOSE
    }
    private IntakeSolenoidState _wantedIntakeSolenoidState = IntakeSolenoidState.CLOSE;

    public synchronized void setWantedIntakeState(IntakeState _intakeState) {//设定intake状态
        _wantedIntakeState = _intakeState;
    }

    public synchronized IntakeState getWantedIntakeState() {//获取intake状态
        return _wantedIntakeState ;
    }
    public synchronized void setWantedIntakeSolenoidState(IntakeSolenoidState _intakeSolenoidState) {//设定气动杆状态
        _wantedIntakeSolenoidState = _intakeSolenoidState;
    }

    public synchronized IntakeSolenoidState getWantedIntakeSolenoidState(){//获取气动杆状态
        return _wantedIntakeSolenoidState;
    }

    private void setIntakeMotor(double speed) {//设定intake电机转速
        m_IntakeMotorLeft.set(ControlMode.PercentOutput, speed);
        m_IntakeMotorRight.set(ControlMode.PercentOutput, speed);
    }

    private void setIntakeState(IntakeState intakeState) {//将intake状态输出到电机上
        switch (intakeState) {
            case INTAKE:
                setIntakeMotor(IntakeConstants.INTAKE_SPEED);
                m_IntakeSolenoid.set(true);
                break;
            case OFF:
                setIntakeMotor(0);
                m_IntakeSolenoid.set(false);
                break;
            case EJECT:
                setIntakeMotor(-IntakeConstants.INTAKE_SPEED);
                m_IntakeSolenoid.set(true);
            default:
                break;
        }
    }

    public boolean isIntakerOn(){
        return getWantedIntakeState() == IntakeState.INTAKE ?true:false;
    }

    @Override
    public void periodic() {
        setIntakeState(_wantedIntakeState);
        OutPutTelemetry();
    }   
}

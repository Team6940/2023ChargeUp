package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.FeedbackDevice;//用不到这些
//import com.ctre.phoenix.motorcontrol.TalonFXControlMode;//这行和下一行只要import一个就可以了
import com.ctre.phoenix.motorcontrol.can.TalonFX;

//import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;





public class Intake extends SubsystemBase {
    public static Intake instance = null;
    public static boolean m_IntakeEnabled=false;
    private TalonFX m_IntakeMotorLeft;
    private TalonFX m_IntakeMotorRight;

    //private PIDController m_IntakeMotorInLeftPIDCOntroller=new PIDController(IntakeConstants.IntakeMotorInkP,IntakeConstants.IntakeMotorInkI,IntakeConstants.IntakeMotorInkD);
    //private PIDController m_IntakeMotorOutLeftPIDCOntroller=new PIDController(IntakeConstants.IntakeMotorOutkP,IntakeConstants.IntakeMotorOutkI,IntakeConstants.IntakeMotorOutkD);

    //private PIDController m_IntakeMotorInRightPIDCOntroller=new PIDController(IntakeConstants.IntakeMotorInkP,IntakeConstants.IntakeMotorInkI,IntakeConstants.IntakeMotorInkD);
    //private PIDController m_IntakeMotorOutRightPIDCOntroller=new PIDController(IntakeConstants.IntakeMotorOutkP,IntakeConstants.IntakeMotorOutkI,IntakeConstants.IntakeMotorOutkD);

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

    }

    public void OutPutTelemetry(){
        SmartDashboard.putNumber("Debug/Intake/CMotorOutput: ", m_IntakeMotorLeft.getMotorOutputPercent());//在调试板上显示电机输出百分比
        SmartDashboard.putNumber("Debug/Intake/CMotorOutput: ", m_IntakeMotorRight.getMotorOutputPercent());//在调试板上显示电机输出百分比
        SmartDashboard.putString("Debug/Intake/WantedIntake State", getWantedIntakeState().name());//在调试版上输出intake的目标状态
    }

    public enum IntakeState {
        INTAKE, EJECT,  OFF
    }
    private IntakeState wantedIntakeState = IntakeState.OFF;

    public synchronized void setWantedIntakeState(IntakeState intakeState) {//设定intake状态
        wantedIntakeState = intakeState;
    }

    public synchronized IntakeState getWantedIntakeState() {//获取intake状态
        return wantedIntakeState ;
    }

    private void setIntakeMotor(double speed) {//设定intake电机转速
        m_IntakeMotorLeft.set(ControlMode.PercentOutput, speed);
        m_IntakeMotorRight.set(ControlMode.PercentOutput, speed);
    }

    private void setIntakeState(IntakeState intakeState) {//将intake状态输出到电机上
        switch (intakeState) {
            case INTAKE:
                setIntakeMotor(IntakeConstants.INTAKE_SPEED);
                break;
            case OFF:
                setIntakeMotor(0);
                break;
            case EJECT:
                setIntakeMotor(-IntakeConstants.INTAKE_SPEED);
            default:
                break;
        }
    }

    public void runIntaker() {
        setWantedIntakeState(IntakeState.INTAKE);
    }

    public void stopIntaker() {
        setWantedIntakeState(IntakeState.OFF);
    }
    public void ejectIntaker(){
        setWantedIntakeState(IntakeState.EJECT);
    }

    private int cnt = 0;
    public void autoturnintaker()//用cnt周期性地控制intake状态
    {
        if(cnt % 2 == 0){
            
            setWantedIntakeState(IntakeState.INTAKE);
        }
        else{
            setWantedIntakeState(IntakeState.OFF);
        }
        cnt++;
    }

    public boolean isIntakerOn(){
        return getWantedIntakeState() == IntakeState.INTAKE ?true:false;
    }

    @Override
    public void periodic() {
        setIntakeState(wantedIntakeState);//持续地将Intake状态输出到电机上
        OutPutTelemetry();
    }   
}

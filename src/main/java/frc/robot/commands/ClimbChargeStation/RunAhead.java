package frc.robot.commands.ClimbChargeStation;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.GameConstants;

public class RunAhead extends CommandBase
{
    
    public RunAhead()
    {
        addRequirements(RobotContainer.m_SwerveBase);
    }

    @Override
    public void initialize()
    {
        
    }
    @Override
    public void execute()
    {
        RobotContainer.m_SwerveBase.Drive(new Translation2d(1,0), 0, true, true);
    }
    @Override
    public boolean isFinished()
    {
        double _Degrees[]={0,0,0,0};
        RobotContainer.m_SwerveBase.m_Gyro.getAccumGyro(_Degrees);
        if(_Degrees[2]<=-GameConstants.ChargeStationLeanDegreesWhenDown)
        {
            return true;
        }
        else
            return false;
    }
}

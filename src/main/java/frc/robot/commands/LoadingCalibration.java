package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class LoadingCalibration extends CommandBase{
    
    public Enum gamePiece;
    public LoadingCalibration()
    {
        addRequirements(RobotContainer.m_SwerveBase);
    }

}

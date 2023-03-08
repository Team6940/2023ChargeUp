package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class GamePieceFocusControl extends CommandBase{
    
    public Enum gamePiece;
    public GamePieceFocusControl()
    {
        addRequirements(RobotContainer.m_SwerveBase);
        addRequirements(RobotContainer.m_Pixy2);
    }

}

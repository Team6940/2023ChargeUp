package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.ClimbChargeStation.KeepBalance;

public class CenterControl extends CommandBase {
    // Alliance m_Alliance= DriverStation.getAlliance();
    static int m_SelectedGrid=0;
    static Command NowCommand;
    public CenterControl()
    {
        addRequirements(RobotContainer.m_Claw);
        addRequirements(RobotContainer.m_Arm);
    }
    @Override
    public void initialize()
    {
        SmartDashboard.putBoolean("rrrr", true);

    }
    @Override
    public void execute()
    {
          }
}

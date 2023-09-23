package frc.robot.commands.autos;

import java.util.List;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.swerve.SwerveDrivetrain;


public class Auto3PieceShort extends SequentialCommandGroup {
    public Auto3PieceShort(SwerveAutoBuilder autoBuilder, SwerveDrivetrain swerve) {
        List<PathPlannerTrajectory> pathGroup = PathPlannerAutos.getPathGroup("3Piece Short");
        
        addCommands(
            Commands.sequence(
                Commands.runOnce(() -> swerve.getImu().zeroAll()),
                autoBuilder.resetPose(pathGroup.get(0))
            )
        );
    }
    
}

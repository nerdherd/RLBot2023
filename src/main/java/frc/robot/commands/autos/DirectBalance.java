package frc.robot.commands.autos;

import java.util.List;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.TheGreatBalancingAct;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class DirectBalance extends SequentialCommandGroup {
    public DirectBalance(SwerveAutoBuilder autoBuilder, SwerveDrivetrain swerve) {
        List<PathPlannerTrajectory> pathGroup = PathPlannerAutos.getPathGroup("DirectBalance");
        
        addCommands(
            Commands.sequence(
                Commands.deadline(
                    new WaitCommand(14.5),
                    Commands.sequence(
                        Commands.runOnce(() -> swerve.getImu().zeroAll()),
                        autoBuilder.resetPose(pathGroup.get(0)),
                        autoBuilder.followPathWithEvents(pathGroup.get(0)),
                        new TheGreatBalancingAct(swerve)
                    ),
                    new InstantCommand(() -> swerve.towModules())
                )
            )
        );
    }
    
}

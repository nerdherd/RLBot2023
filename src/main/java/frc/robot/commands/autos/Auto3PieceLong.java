package frc.robot.commands.autos;

import java.util.List;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.swerve.SwerveDrivetrain;


public class Auto3PieceLong extends SequentialCommandGroup {
    public Auto3PieceLong(SwerveAutoBuilder autoBuilder,  SwerveDrivetrain swerve) {
        List<PathPlannerTrajectory> pathGroup = PathPlannerAutos.getPathGroup("3Piece Long");
        
        addCommands(
            Commands.sequence(
                Commands.runOnce(() -> swerve.getImu().zeroAll()),
                autoBuilder.resetPose(pathGroup.get(0))
                // wrist.motionMagicCommand(WristConstants.kWristHigh)),
                // shoot.outtakeHigh(),
                // Commands.parallel(
                //     wrist.motionMagicCommand(WristConstants.kWristStow),
                //     autoBuilder.followPathWithEvents(pathGroup.get(0))
                // ),
                // Commands.parallel(
                //     wrist.motionMagicCommand(WristConstants.kWristGround),
                //     autoBuilder.followPathWithEvents(pathGroup.get(1)),
                //     shoot.setPower(ShooterConstants.kTopIntakePower.get(),ShooterConstants.kBottomIntakePower.get())
                // ),
                // Commands.parallel(
                //     wrist.motionMagicCommand(WristConstants.kWristStow),
                //     autoBuilder.followPathWithEvents(pathGroup.get(2)),
                //     shoot.setPower(ShooterConstants.kTopIntakeNeutralPower.get(),ShooterConstants.kBottomIntakeNeutralPower.get())
                // ),
                // shoot.setPower(ShooterConstants.kTopIntakeNeutralPower.get(),ShooterConstants.kBottomIntakeNeutralPower.get()),
                // autoBuilder.followPathWithEvents(pathGroup.get(3)),
                // wrist.motionMagicCommand(WristConstants.kWristLow),
                // shoot.outtakeLow(),
                // Commands.parallel(
                //     wrist.motionMagicCommand(WristConstants.kWristStow),
                //     autoBuilder.followPathWithEvents(pathGroup.get(4))
                // ),
                // Commands.parallel(
                //     wrist.motionMagicCommand(WristConstants.kWristGround),
                //     autoBuilder.followPathWithEvents(pathGroup.get(5)),
                //     shoot.setPower(ShooterConstants.kTopIntakePower.get(),ShooterConstants.kBottomIntakePower.get())
                // ),
                // Commands.parallel(
                //     wrist.motionMagicCommand(WristConstants.kWristStow),
                //     autoBuilder.followPathWithEvents(pathGroup.get(6)),
                //     shoot.setPower(ShooterConstants.kTopIntakeNeutralPower.get(),ShooterConstants.kBottomIntakeNeutralPower.get())
                // ),
                // shoot.setPower(ShooterConstants.kTopIntakeNeutralPower.get(),ShooterConstants.kBottomIntakeNeutralPower.get()),
                // autoBuilder.followPathWithEvents(pathGroup.get(7)),
                // wrist.motionMagicCommand(WristConstants.kWristLow),
                // shoot.outtakeLow()
        ));
    }
    
}

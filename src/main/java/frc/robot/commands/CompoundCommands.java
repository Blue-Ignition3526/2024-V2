package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.IndexerPivot;
import frc.robot.subsystems.IndexerRollers;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class CompoundCommands {
    public static Command intakeCommand(IndexerPivot indexerPivot, Elevator elevator, Shooter shooter, Intake intake, IndexerRollers rollers, BooleanSupplier condition) {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                indexerPivot.setSetpointCommand(Constants.Indexer.Pivot.receiving),
                shooter.stopCommand(),
                elevator.setPositionCommand(0)
            ),

            new ParallelCommandGroup(
                rollers.setRollersInCommand(),
                intake.setInCommand()
            ).until(condition).finallyDo(new ParallelCommandGroup(
                new WaitCommand(0.15),
                rollers.stopRollersCommand(),
                intake.setStopCommand()
            )::schedule)
        );
    }

    public static Command outtakeCommand(IndexerPivot indexerPivot, Elevator elevator, Shooter shooter, Intake intake, IndexerRollers rollers) {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                indexerPivot.setSetpointCommand(Constants.Indexer.Pivot.receiving),
                shooter.stopCommand(),
                elevator.setPositionCommand(0)
            ),

            new ParallelCommandGroup(
                rollers.setRollersOutCommand(),
                intake.setOutCommand()
            ).finallyDo(new ParallelCommandGroup(
                rollers.stopRollersCommand(),
                intake.setStopCommand()
            )::schedule)
        );
    }

    public static Command shootCommand(IndexerPivot indexerPivot, IndexerRollers rollers, Shooter shooter, DoubleSupplier angleSupplier) {
        return new ParallelCommandGroup(
            indexerPivot.setSetpointCommand(angleSupplier.getAsDouble()),
            shooter.setShootCommand()
        ).finallyDo(new SequentialCommandGroup(
            rollers.setRollersPassCommand(),

            new WaitCommand(1.25),

            new ParallelCommandGroup(
                shooter.stopCommand(),
                rollers.stopRollersCommand()
            )
        )::schedule);
    }

    public static Command highShootCommand(IndexerPivot indexerPivot, IndexerRollers rollers, Shooter shooter) {
        return CompoundCommands.shootCommand(indexerPivot, rollers, shooter, () -> Constants.Indexer.Pivot.shoot);
    }

    public static Command lowShootCommand(IndexerPivot indexerPivot, IndexerRollers rollers, Shooter shooter) {
        return CompoundCommands.shootCommand(indexerPivot, rollers, shooter, () -> Constants.Indexer.Pivot.horizontal);
    }

    public static Command ampCommand(IndexerPivot indexerPivot, IndexerRollers rollers, Elevator elevator) {
        return new ParallelCommandGroup(
            elevator.setPositionCommand(55).until(elevator::atSetpoint),
            indexerPivot.setSetpointCommand(0).until(indexerPivot::atSetpoint)
        ).finallyDo(new SequentialCommandGroup(
            rollers.setRollersOutCommand(),
            new WaitCommand(1.25),
            rollers.stopRollersCommand(),

            elevator.setPositionCommand(0)
        )::schedule);
    }
}

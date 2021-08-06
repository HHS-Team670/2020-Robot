// package frc.team670.robot.commands.routines;

// import java.util.HashMap;
// import java.util.Map;

// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.team670.mustanglib.commands.MustangCommand;
// import frc.team670.robot.commands.indexer.SendOneBallToShoot;
// import frc.team670.robot.commands.indexer.StageOneBallToShoot;
// import frc.team670.robot.commands.shooter.StartShooter;
// import frc.team670.robot.commands.shooter.StartShooterByDistance;
// import frc.team670.robot.subsystems.DriveBase;
// import frc.team670.robot.subsystems.Indexer;
// import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
// import frc.team670.robot.subsystems.Shooter;
// import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;

// public class RotateIndexerToUptakeThenShoot extends SequentialCommandGroup implements MustangCommand {

//     private Map<MustangSubsystemBase, HealthState> healthReqs;
//     private Indexer indexer;
//     private Shooter shooter;

//     public RotateIndexerToUptakeThenShoot(Indexer indexer, Shooter shooter, DriveBase driveBase) {
//         this.indexer = indexer;
//         this.shooter = shooter;
//         addRequirements(indexer, shooter);
//         healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
//         healthReqs.put(indexer, HealthState.GREEN);
//         healthReqs.put(shooter, HealthState.GREEN);

//         // Stage and uptake 1 ball while preparing the shooter
//         addCommands(
//             new ParallelCommandGroup(
//                 new StartShooterByDistance(shooter, driveBase), 
//                 new StageOneBallToShoot(indexer)
//             )
//             // new SendOneBallToShoot(indexer));
//         );
//     }

//     /**
//      * This sequence will only end if we tell it to, otherwise, we keep blasting
//      */
//     @Override
//     public void end(boolean interrupted) {
//         if (interrupted) {
//             shooter.stop();
//             indexer.stopUpdraw();
//         }
//     }

//     @Override
//     public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
//         return healthReqs;
//     }

// }
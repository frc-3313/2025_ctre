// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CompoundCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.BasicCommands.CoralCMD;
import frc.robot.commands.BasicCommands.ScoreCoralCMD;
import frc.robot.commands.BasicCommands.ElevatorGoToHeightCMD;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.StateMachine;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoScore extends SequentialCommandGroup {
  /** Creates a new AutoScore. */
  public AutoScore(StateMachine stateMachine, Coral coral, Elevator elevator) {
    // Add your commands in the addCommands() call, e.g.
    addCommands(
      new CoralCMD(coral, stateMachine, .15),
      new ElevatorGoToHeightCMD(coral, elevator, stateMachine),
      new ScoreCoralCMD(coral, elevator, stateMachine)
    );
  }
}

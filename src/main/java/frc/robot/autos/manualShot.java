// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.test.Flywheel;
import frc.robot.commands.test.HoodCMD;
import frc.robot.subsystems.ShooterStructure;
import frc.robot.subsystems.Shooter.HoodSub;
import frc.robot.subsystems.Shooter.ShooterSub;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class manualShot extends ParallelCommandGroup {
  /** Creates a new manualShot. */
  public manualShot(ShooterSub m_flywheel, HoodSub m_hood) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //these need to be tuned for wherever we end up shooting from during auto
      //figured it might be too risky to move back up to the hub
      //with everyone moving back and forth
      new Flywheel(m_flywheel, 2000),
      new HoodCMD(m_hood, 15)
    );
  }
}

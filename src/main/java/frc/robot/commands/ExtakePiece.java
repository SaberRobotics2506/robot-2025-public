package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Extake;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/** An example command that uses an example subsystem. */
public class ExtakePiece extends Command {
  
  /**
   * Creates a new AngleArm.
   *
   * @param subsystem The subsystem used by this command.
   */

  Extake extake;

  public ExtakePiece(Extake extake) {
      this.extake = extake;
      addRequirements(extake);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putString("setting", "setting");
  }

  // Called every tnime the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    while (!extake.getIrSensor()) {
      extake.runMotor();
    }

    if (isFinished()) {
      end(false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    extake.stopMotor();
    SmartDashboard.putString("setting", "ending");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (extake.getIrSensor()) {
      return true;
    }
    return false;
  }
}

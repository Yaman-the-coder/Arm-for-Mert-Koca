// Yaman has all the copyright to this code

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.someSystems.ArmForMertKoca;
import java.time.LocalTime;

public class Robot extends TimedRobot {
  private final ArmForMertKoca m_arm = new ArmForMertKoca();

  public Robot() {}

  @Override
  public void simulationPeriodic() {
    m_arm.simulationPeriodic();
  }

  @Override
  public void teleopPeriodic() {
    int second = LocalTime.now().getSecond();
    int lastDigit = second % 10; 
    if (lastDigit < 5) {
      //lift the mert koca's arm
      m_arm.reachSetpoint();
    } else {
      //lower the mert koca's arm
      m_arm.stop();
    }
  }
}
//made with love by yaman(i am not sure if i love this code tho)
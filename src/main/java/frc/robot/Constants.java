package frc.robot;

import edu.wpi.first.math.util.Units;

public class Constants {

  public static final double minimumArmAngleInRadians = Units.degreesToRadians(-75);
  public static final int encoderSignalChannelAForArmPosition = 0;
  public static final double defaultProportionalGainForArmPidController = 50.0;
  public static final int encoderSignalChannelBForArmPosition = 1;
  public static final String keyForArmForMertKocaP = "ArmForMertKocaP";
  public static final double lengthOfTheArmInMetersForSimulation = Units.inchesToMeters(30);
  public static final int pwmPortNumberForMotorController = 0;
  public static final double massOfTheRotatingArmForSimulation = 8.0;
  public static final String keyForArmForMertKocaPos = "ArmForMertKocaPos";
  public static final double memberVariableThatStoresTheTargetRotationAngleOfTheArmInDegreesUsedByThePidController = 75.0;
  public static final double memberVariableThatDefinesTheMaximumAllowedRotationAngleOfTheArmInRadiansForTheSimulation = Units.degreesToRadians(255);
  public static final double armRotationDistanceOrAnglePerEncoderPulse = 2.0 * Math.PI / 4096;
  public static final double reductionRatioOfTheArmGearbox = 200;
}

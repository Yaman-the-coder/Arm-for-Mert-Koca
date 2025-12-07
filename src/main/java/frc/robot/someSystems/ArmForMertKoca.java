package frc.robot.someSystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class ArmForMertKoca implements AutoCloseable {
    private double kp = Constants.defaultProportionalGainForArmPidController;
    private double setDeg = Constants.memberVariableThatStoresTheTargetRotationAngleOfTheArmInDegreesUsedByThePidController;
    private final PWMSparkMax pwmMotor = new PWMSparkMax(Constants.pwmPortNumberForMotorController);
    private final Encoder enc = new Encoder(Constants.encoderSignalChannelAForArmPosition, Constants.encoderSignalChannelBForArmPosition);
    private final PIDController pid = new PIDController(kp, 0, 0);
    private final DCMotor gearbox = DCMotor.getVex775Pro(2);
    private final SingleJointedArmSim sim = new SingleJointedArmSim(
            gearbox,
            Constants.reductionRatioOfTheArmGearbox,
            SingleJointedArmSim.estimateMOI(Constants.lengthOfTheArmInMetersForSimulation, Constants.massOfTheRotatingArmForSimulation),
            Constants.lengthOfTheArmInMetersForSimulation,
            Constants.minimumArmAngleInRadians,
            Constants.memberVariableThatDefinesTheMaximumAllowedRotationAngleOfTheArmInRadiansForTheSimulation,
            true,
            0,
            Constants.armRotationDistanceOrAnglePerEncoderPulse,
            0.0
    );
    private final EncoderSim simEnc = new EncoderSim(enc);
    private final Mechanism2d mech = new Mechanism2d(60, 60);
    private final MechanismRoot2d pivot = mech.getRoot("pivot", 30, 30);
    private final MechanismLigament2d tower = pivot.append(new MechanismLigament2d("tower", 30, -90));
    private final MechanismLigament2d arm = pivot.append(
            new MechanismLigament2d(
                    "ArmForMertKoca",
                    30,
                    Units.radiansToDegrees(sim.getAngleRads()),
                    6,
                    new Color8Bit(Color.kBrown)
            )
    );
    public ArmForMertKoca() {
        enc.setDistancePerPulse(Constants.armRotationDistanceOrAnglePerEncoderPulse);
        SmartDashboard.putData("Arm Sim", mech);
        tower.setColor(new Color8Bit(Color.kCyan));
        Preferences.initDouble(Constants.keyForArmForMertKocaPos, setDeg);
        Preferences.initDouble(Constants.keyForArmForMertKocaP, kp);
    }
    public void loadPreferences() {
        setDeg = Preferences.getDouble(Constants.keyForArmForMertKocaPos, setDeg);
        double newKp = Preferences.getDouble(Constants.keyForArmForMertKocaP, kp);
        if (kp != newKp) {
            kp = newKp;
            pid.setP(kp);
        }
    }
    public void simulationPeriodic() {
        sim.setInput(pwmMotor.get() * RobotController.getBatteryVoltage());
        sim.update(0.020);
        simEnc.setDistance(sim.getAngleRads());
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(sim.getCurrentDrawAmps()));
        arm.setAngle(Units.radiansToDegrees(sim.getAngleRads()));
    }
    public void reachSetpoint() {
        double output = pid.calculate(enc.getDistance(), Units.degreesToRadians(setDeg));
        pwmMotor.setVoltage(output);
    }
    public void stop() {
        pwmMotor.set(0.0);
    }
    @Override
    public void close() {
        pwmMotor.close();
        enc.close();
        mech.close();
        pivot.close();
        pid.close();
        arm.close();
    }
}
//made with love by yaman(i am not sure if i love this code tho)
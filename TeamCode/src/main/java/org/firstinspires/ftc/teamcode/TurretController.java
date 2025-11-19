package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Configurable
public class TurretController {

    // --- Hardware ---
    private DcMotorEx turretMotor;
    private Limelight3A limelight;
    private IMU imu;
    private Telemetry telemetry;

    // --- Constants ---
    private static final double TICKS_PER_DEGREE = (28 * 20 * 95.0 / 28.0) / 360.0;
    public static double P = 15.0, I = 0.0, D = 0.0, F = 0.0;
    public static double TURRET_MAX_POWER = 0.8;
    public static double TARGET_LOST_TIMEOUT = 2.0;

    public static int LIMELIGHT_PIPELINE = 8; // this should probably stay like this

    // --- State ---
    private boolean autoAimEnabled = false;
    private double manualTargetAngle = 0.0;
    private double lastKnownTargetAngleField = 0.0;
    private boolean targetWasVisible = false;
    private final ElapsedTime targetLostTimer = new ElapsedTime();

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        try {
            // Initialize the IMU with the modern API.
            imu = hardwareMap.get(IMU.class, "imu");
            // TODO: Adjust the orientation parameters to match your robot's configuration.
            // This is for a Control Hub that is mounted horizontally with the logo facing up and the USB ports facing forward.
            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
            imu.initialize(parameters);

            turretMotor = hardwareMap.get(DcMotorEx.class, "aim");
            turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            turretMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(P, I, D, F));

            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(LIMELIGHT_PIPELINE);
            limelight.start();

            targetLostTimer.reset();
            telemetry.addData("Turret Controller", "Initialized");
        } catch (Exception e) {
            telemetry.addData("Turret Init Error", e.getMessage());
        }
    }

    public void update() {
        if (autoAimEnabled) {
            updateAutoAim();
        } else {
            updateManualControl();
        }
        updateTelemetry();
    }

    private void updateAutoAim() {
        double robotHeading = getRobotHeading();
        double currentTurretAngle = getCurrentAngle();

        LLResult result = limelight.getLatestResult();
        if (hasValidTarget(result)) {
            double tx = result.getFiducialResults().get(0).getTargetXDegrees();
            lastKnownTargetAngleField = robotHeading + currentTurretAngle + tx;
            targetWasVisible = true;
            targetLostTimer.reset();
        }

        if (targetWasVisible && targetLostTimer.seconds() < TARGET_LOST_TIMEOUT) {
            double desiredTurretAngle = lastKnownTargetAngleField - robotHeading;
            setTargetAngleInternal(desiredTurretAngle);
        } else {
            turretMotor.setPower(0);
            targetWasVisible = false;
        }
    }

    private void updateManualControl() {
        setTargetAngleInternal(manualTargetAngle);
    }

    private void setTargetAngleInternal(double angle) {
        int targetPositionTicks = (int) (angle * TICKS_PER_DEGREE);
        turretMotor.setTargetPosition(targetPositionTicks);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(TURRET_MAX_POWER);
    }

    public void enableAutoAim(boolean enable) {
        this.autoAimEnabled = enable;
    }

    public void setManualTargetAngle(double angle) {
        this.manualTargetAngle = angle;
    }

    public double getCurrentAngle() {
        return turretMotor.getCurrentPosition() / TICKS_PER_DEGREE;
    }

    public boolean isTracking() {
        return autoAimEnabled && targetWasVisible;
    }

    /**
     * Gets the robot's current heading (yaw) from the IMU.
     * @return The robot's heading in degrees.
     */
    private double getRobotHeading() {
        YawPitchRollAngles robotOrientation = imu.getRobotYawPitchRollAngles();
        return robotOrientation.getYaw(AngleUnit.DEGREES);
    }

    private boolean hasValidTarget(LLResult result) {
        if (result == null || !result.isValid()) return false;
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        return fiducials != null && !fiducials.isEmpty();
    }

    public void stop() {
        if (turretMotor != null) turretMotor.setPower(0);
        if (limelight != null) limelight.stop();
    }

    public void reset() {
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        imu.resetYaw(); // Reset the IMU's yaw when resetting the turret
        manualTargetAngle = 0;
    }

    private void updateTelemetry() {
        telemetry.addData("Turret Mode", autoAimEnabled ? "Auto-Aim" : "Manual");
        telemetry.addData("Turret Angle", "%.1f°", getCurrentAngle());
        telemetry.addData("Robot Heading", "%.1f°", getRobotHeading());
        if(autoAimEnabled) {
            telemetry.addData("Tracking Status", isTracking() ? (targetLostTimer.seconds() > 0.1 ? "IMU" : "Vision") : "Searching");
        }
    }
}

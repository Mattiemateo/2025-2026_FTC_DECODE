package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@SuppressWarnings({"unused", "FieldCanBeLocal"})
@TeleOp(name = "Auto-aiming turret v2", group = "Iterative Opmode")
public class AutoAimingTurret extends OpMode {

    // --- Hardware ---
    private DcMotorEx turretMotor;
    private Limelight3A limelight;
    private BNO055IMU imu;

    // --- Constants ---

    // TODO: Tune this value for your specific turret motor and gear ratio.
    // This is for a GoBILDA 5203 series motor (537.7 Ticks/Rev) with a 1:1 gear ratio.
    private static final double TICKS_PER_DEGREE = 537.7 / 360.0;

    // PIDF Coefficients for the turret motor's RUN_TO_POSITION mode.
    // TODO: Tune these coefficients for your turret for snappy and accurate movements.
    private static final double P = 10.0; // Proportional - start around 10
    private static final double I = 0.0; // Integral - keep 0 for now
    private static final double D = 0.0; // Derivative - keep 0 for now
    private static final double F = 0.0; // Feedforward - can be 0

    // Maximum power for the turret motor when moving to a position.
    private static final double MAX_POWER = 0.8;

    // How long to keep tracking with the IMU after the target is lost.
    private static final double TARGET_LOST_TIMEOUT = 2.0; // seconds

    // --- State Variables ---
    // The field-centric angle of the target, which we will continuously aim at.
    private double lastKnownTargetAngleField = 0.0;
    private boolean targetWasVisible = false;
    private final ElapsedTime targetLostTimer = new ElapsedTime();

    @Override
    public void init() {
        try {
            // --- IMU Initialization ---
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            imu.initialize(parameters);

            // --- Turret Motor Initialization ---
            turretMotor = hardwareMap.get(DcMotorEx.class, "aim");
            turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Start in velocity control

            // Set the PIDF coefficients for position control
            PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, I, D, F);
            turretMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidfCoefficients);

            // --- Limelight Initialization ---
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(0); // Use your AprilTag pipeline
            limelight.start();

            targetLostTimer.reset();
            telemetry.addData("Status", "Initialized");
        } catch (Exception e) {
            telemetry.addData("Init Error", e.getMessage());
        }
    }

    @Override
    public void loop() {
        try {
            // --- Get Current State ---
            double robotHeading = getRobotHeading();
            double currentTurretAngle = turretMotor.getCurrentPosition() / TICKS_PER_DEGREE;

            // --- Vision Processing ---
            LLResult result = limelight.getLatestResult();
            if (hasValidTarget(result)) {
                // TARGET IS VISIBLE: Recalculate the field-centric angle.
                double tx = result.getFiducialResults().get(0).getTargetXDegrees();

                // Field Angle = Robot's Angle + Turret's Angle relative to Robot + Target's Angle relative to Turret
                lastKnownTargetAngleField = robotHeading + currentTurretAngle + tx;

                targetWasVisible = true;
                targetLostTimer.reset(); // Reset the timer since we found the target
            }

            // --- Control Logic ---
            if (targetWasVisible && targetLostTimer.seconds() < TARGET_LOST_TIMEOUT) {
                // AIM AT TARGET: We have a target to aim at (either visible now or recently).
                // Calculate the desired turret angle to counteract robot rotation.
                double desiredTurretAngle = lastKnownTargetAngleField - robotHeading;

                // Convert the desired angle to motor encoder ticks.
                int targetPositionTicks = (int) (desiredTurretAngle * TICKS_PER_DEGREE);

                // Command the motor to move to the target position.
                turretMotor.setTargetPosition(targetPositionTicks);
                turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                turretMotor.setPower(MAX_POWER); // Apply power to move to the position

                telemetry.addData("Status", targetLostTimer.seconds() > 0.1 ? "TRACKING (IMU)" : "LOCKED ON (Vision)");
                telemetry.addData("Desired Turret Angle", "%.2f", desiredTurretAngle);

            } else {
                // TARGET LOST: Stop the turret, it will hold its last position due to BRAKE behavior.
                turretMotor.setPower(0);
                targetWasVisible = false; // Give up until we see the target again
                telemetry.addData("Status", "SEARCHING");
            }

            // --- Telemetry ---
            telemetry.addData("Robot Heading", "%.2f", robotHeading);
            telemetry.addData("Current Turret Angle", "%.2f", currentTurretAngle);
            telemetry.addData("Last Field Angle", "%.2f", lastKnownTargetAngleField);
            telemetry.addData("Target Visible", hasValidTarget(result) ? "Yes" : "No");

        } catch (Exception e) {
            telemetry.addData("Loop Error", e.getMessage());
            turretMotor.setPower(0); // Stop motor on error
        }
        telemetry.update();
    }

    /**
     * Gets the robot's current heading from the IMU.
     * @return The robot's heading in degrees, from -180 to 180.
     */
    private double getRobotHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle; // Usually Z axis is the heading
    }

    /**
     * Checks if the Limelight result is valid and contains at least one AprilTag.
     * @param result The LLResult from the Limelight.
     * @return True if a valid target is found, false otherwise.
     */
    private boolean hasValidTarget(LLResult result) {
        if (result == null || !result.isValid()) {
            return false;
        }
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        return fiducials != null && !fiducials.isEmpty();
    }

    @Override
    public void stop() {
        try {
            if (turretMotor != null) {
                turretMotor.setPower(0);
            }
            if (limelight != null) {
                limelight.stop();
            }
        } catch (Exception e) {
            // Fail silently on stop
        }
    }
}

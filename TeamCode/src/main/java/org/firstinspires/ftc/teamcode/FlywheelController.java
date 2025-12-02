package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Configurable
public class FlywheelController {

    // --- Hardware ---
    private DcMotorEx flywheelMotor;
    private Telemetry telemetry;

    // --- Motor Constants ---
    // Encoder ticks per revolution for your motor
    // Adjust TICKS_PER_REV based on your motor and any gearing
    public static double TICKS_PER_REV = 52.0; // Base encoder ticks per revolution

    // --- PID Coefficients ---
    // These will need tuning for the flywheel
    // F (feedforward) is probably most important for velocity control
    public static double FLYWHEEL_P = 10.0;
    public static double FLYWHEEL_I = 0.5;
    public static double FLYWHEEL_D = 0.0;
    public static double FLYWHEEL_F = 12.0; // Feedforward - main driver for velocity

    // --- Flywheel Settings ---
    public static double IDLE_RPM = 0.0; // RPM when flywheel is "off"
    public static double DEFAULT_RPM = 3000.0; // Default target RPM when spinning up
    public static double MAX_RPM = 6000.0; // Maximum safe RPM

    // --- Distance to RPM Lookup Table ---
    // Format: {distance in meters, target RPM}
    // Add more entries as you test and calibrate
    private static final double[][] DISTANCE_TO_RPM_TABLE = {
        { 0.5, 2000.0 }, // 0.5m -> 2000 RPM
        { 1.0, 2500.0 }, // 1.0m -> 2500 RPM
        { 1.5, 3000.0 }, // 1.5m -> 3000 RPM
        { 2.0, 3500.0 }, // 2.0m -> 3500 RPM
        { 2.5, 4000.0 }, // 2.5m -> 4000 RPM
        { 3.0, 4500.0 }, // 3.0m -> 4500 RPM
        { 3.5, 5000.0 }, // 3.5m -> 5000 RPM
        { 4.0, 5500.0 }, // 4.0m -> 5500 RPM
    };

    // --- State ---
    private double targetRPM = 0.0;
    private boolean enabled = false;
    private boolean useDistanceLookup = false;

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        try {
            flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheel");

            // Reset encoder
            flywheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Set direction (adjust based on your setup)
            flywheelMotor.setDirection(DcMotor.Direction.REVERSE);

            // Use FLOAT so flywheel can coast when power is cut
            flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            // Set PIDF coefficients for velocity control
            flywheelMotor.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(FLYWHEEL_P, FLYWHEEL_I, FLYWHEEL_D, FLYWHEEL_F)
            );

            // Set to velocity control mode
            flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            telemetry.addData("Flywheel Controller", "Initialized");
        } catch (Exception e) {
            telemetry.addData("Flywheel Init Error", e.getMessage());
        }
    }

    /**
     * Call this every loop iteration to update flywheel control.
     */
    public void update() {
        if (flywheelMotor == null) return;

        if (enabled) {
            // Convert RPM to ticks per second
            double ticksPerSecond = rpmToTicksPerSecond(targetRPM);
            flywheelMotor.setVelocity(ticksPerSecond);
        } else {
            flywheelMotor.setVelocity(0);
        }
    }

    /**
     * Enable or disable the flywheel.
     */
    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
        if (!enabled) {
            targetRPM = IDLE_RPM;
        }
    }

    /**
     * Set the target RPM directly.
     */
    public void setTargetRPM(double rpm) {
        this.targetRPM = Math.min(Math.max(rpm, 0), MAX_RPM);
        this.useDistanceLookup = false;
    }

    /**
     * Set target RPM based on distance using the lookup table.
     * Uses linear interpolation between table entries.
     */
    public void setTargetFromDistance(double distanceMeters) {
        this.targetRPM = interpolateRPM(distanceMeters);
        this.useDistanceLookup = true;
    }

    /**
     * Spin up to the default RPM.
     */
    public void spinUp() {
        setEnabled(true);
        if (!useDistanceLookup) {
            setTargetRPM(DEFAULT_RPM);
        }
    }

    /**
     * Stop the flywheel (let it coast).
     */
    public void stop() {
        setEnabled(false);
    }

    /**
     * Temporarily cut power (for flipper launch).
     * Call this when the flipper needs extra power.
     */
    public void interruptPower() {
        if (flywheelMotor != null) {
            flywheelMotor.setPower(0);
        }
    }

    /**
     * Resume normal velocity control after interrupt.
     */
    public void resumePower() {
        if (flywheelMotor != null) {
            flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * Get the current flywheel RPM from the encoder.
     */
    public double getCurrentRPM() {
        if (flywheelMotor == null) return 0.0;
        double ticksPerSecond = flywheelMotor.getVelocity();
        return ticksPerSecondToRPM(ticksPerSecond);
    }

    /**
     * Get the target RPM.
     */
    public double getTargetRPM() {
        return targetRPM;
    }

    /**
     * Check if the flywheel is at target speed (within tolerance).
     */
    public boolean isAtTargetSpeed() {
        double tolerance = 100.0; // RPM tolerance
        return Math.abs(getCurrentRPM() - targetRPM) < tolerance;
    }

    /**
     * Check if flywheel is enabled.
     */
    public boolean isEnabled() {
        return enabled;
    }

    /**
     * Linear interpolation to get RPM for a given distance.
     */
    private double interpolateRPM(double distance) {
        // Handle edge cases
        if (distance <= DISTANCE_TO_RPM_TABLE[0][0]) {
            return DISTANCE_TO_RPM_TABLE[0][1];
        }
        if (
            distance >=
            DISTANCE_TO_RPM_TABLE[DISTANCE_TO_RPM_TABLE.length - 1][0]
        ) {
            return DISTANCE_TO_RPM_TABLE[DISTANCE_TO_RPM_TABLE.length - 1][1];
        }

        // Find the two points to interpolate between
        for (int i = 0; i < DISTANCE_TO_RPM_TABLE.length - 1; i++) {
            double d1 = DISTANCE_TO_RPM_TABLE[i][0];
            double d2 = DISTANCE_TO_RPM_TABLE[i + 1][0];

            if (distance >= d1 && distance <= d2) {
                double rpm1 = DISTANCE_TO_RPM_TABLE[i][1];
                double rpm2 = DISTANCE_TO_RPM_TABLE[i + 1][1];

                // Linear interpolation
                double t = (distance - d1) / (d2 - d1);
                return rpm1 + t * (rpm2 - rpm1);
            }
        }

        return DEFAULT_RPM;
    }

    /**
     * Convert RPM to encoder ticks per second.
     */
    private double rpmToTicksPerSecond(double rpm) {
        // RPM -> revolutions per second -> ticks per second
        return (rpm / 60.0) * TICKS_PER_REV;
    }

    /**
     * Convert encoder ticks per second to RPM.
     */
    private double ticksPerSecondToRPM(double ticksPerSecond) {
        // Ticks per second -> revolutions per second -> RPM
        return (ticksPerSecond / TICKS_PER_REV) * 60.0;
    }

    /**
     * Update telemetry with flywheel status.
     */
    public void updateTelemetry() {
        if (telemetry == null) return;

        telemetry.addData("Flywheel Enabled", enabled);
        telemetry.addData("Flywheel Target RPM", "%.0f", targetRPM);
        telemetry.addData("Flywheel Current RPM", "%.0f", getCurrentRPM());
        if (enabled) {
            telemetry.addData(
                "Flywheel At Speed",
                isAtTargetSpeed() ? "YES" : "Spinning up..."
            );
        }
    }
}

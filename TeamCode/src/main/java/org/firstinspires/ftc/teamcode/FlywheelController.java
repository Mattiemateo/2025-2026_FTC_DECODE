package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class FlywheelController {
    // All RPM values in this class and in ShotTable are FLYWHEEL RPM, not motor RPM.
    // Encoder ticks per MOTOR revolution (last season's calibration, motor was direct to the flywheel).
    public static double MOTOR_TICKS_PER_REV = 52.0;
    // Flywheel revolutions per motor revolution. 2026-11-08: geared up 2:1 for more flywheel speed.
    public static double GEAR_RATIO = 2.0;
    public static double DEFAULT_RPM = 2000.0;
    public static double MAX_RPM = 6000.0; // flywheel
    public static double FLYWHEEL_P = 9.0;
    public static double FLYWHEEL_I = 0.5;
    public static double FLYWHEEL_D = 0.0;
    public static double FLYWHEEL_F = 15.0;

    private final DcMotorEx motor;
    private double targetRPM;

    public FlywheelController(DcMotorEx motor) {
        this.motor = motor;
        motor.setPower(0);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(FLYWHEEL_P, FLYWHEEL_I, FLYWHEEL_D, FLYWHEEL_F));
    }

    public void setTargetRPM(double rpm) {
        double ticksPerRev = ticksPerFlywheelRev();
        if (!Double.isFinite(rpm) || !Double.isFinite(ticksPerRev) || ticksPerRev <= 0) {
            throw new IllegalArgumentException("Flywheel RPM and encoder calibration must be valid");
        }
        targetRPM = Math.max(0, Math.min(rpm, MAX_RPM));
        if (targetRPM == 0) motor.setPower(0); // Cut power so FLOAT can coast instead of braking to zero RPM.
        else motor.setVelocity(targetRPM * ticksPerRev / 60.0);
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public double getCurrentRPM() {
        return motor.getVelocity() * 60.0 / ticksPerFlywheelRev();
    }

    /** Motor encoder ticks per flywheel revolution, through the gearing. */
    static double ticksPerFlywheelRev() {
        return MOTOR_TICKS_PER_REV / GEAR_RATIO;
    }
}

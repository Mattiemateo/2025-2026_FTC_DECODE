package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class TurretController {
    private DcMotorEx turretMotor;

    // PID Constants
    private double kP = 0.15;
    private double kI = 0.0;
    private double kD = 0.0002;

    // PID Variables
    private double integralSum = 0;
    private double lastError = 0;
    private ElapsedTime timer = new ElapsedTime();

    // Turret Configuration
    // 28 ticks/rev * 20:1 planetary * (95/28) gear reduction = 1900 ticks/rev
    private static final double TICKS_PER_DEGREE = (28 * 20 * 95.0 / 28.0) / 360.0;
    private static final double MAX_POWER = 0.5;
    private static final double TOLERANCE = 2.0; // degrees

    private double targetAngle = 0;

    public void init(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotorEx.class, "aim");// port 1
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        timer.reset();
    }

    /**
     * Update PID loop - call this every loop iteration
     */
    public void update() {
        double currentAngle = getCurrentAngle();
        double error = targetAngle - currentAngle;

        // Normalize error to [-180, 180]
        while (error > 180) error -= 360;
        while (error < -180) error += 360;

        double deltaTime = timer.seconds();
        timer.reset();

        // Calculate PID terms
        integralSum += error * deltaTime;
        double derivative = (error - lastError) / deltaTime;
        double power = (kP * error) + (kI * integralSum) + (kD * derivative);

        // Clamp power to safe limits
        power = Math.max(-MAX_POWER, Math.min(MAX_POWER, power));

        turretMotor.setPower(power);
        lastError = error;
    }

    /**
     * Set target angle in degrees
     */
    public void setTargetAngle(double angle) {
        targetAngle = angle;
        integralSum = 0; // Reset integral on new target
    }

    /**
     * Get current turret angle in degrees
     */
    public double getCurrentAngle() {
        return turretMotor.getCurrentPosition() / TICKS_PER_DEGREE;
    }

    /**
     * Check if turret is at target
     */
    public boolean isAtTarget() {
        return Math.abs(targetAngle - getCurrentAngle()) < TOLERANCE;
    }

    /**
     * Adjust PID constants for tuning
     */
    public void setPID(double p, double i, double d) {
        kP = p;
        kI = i;
        kD = d;
    }

    /**
     * Reset encoder to zero
     */
    public void reset() {
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        targetAngle = 0;
        integralSum = 0;
        lastError = 0;
    }

    /**
     * Stop turret movement
     */
    public void stop() {
        turretMotor.setPower(0);
    }
}
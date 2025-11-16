package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Limelight Auto-Aim", group = "Iterative Opmode")
public class LimelightAutoAim extends OpMode {

    // Hardware
    private DcMotorEx turretMotor;
    private Limelight3A limelight;

    // PID Controller variables
    private double kP = 0.015; // Proportional gain - START LOW and increase
    private double kI = 0.0001; // Integral gain - usually keep small
    private double kD = 0.002; // Derivative gain - helps reduce overshoot

    private double targetX = 0.0; // Target is centered (tx = 0)
    private double integral = 0.0;
    private double lastError = 0.0;
    private ElapsedTime timer = new ElapsedTime();

    // Deadband and limits
    private static final double POSITION_TOLERANCE = 1.5; // degrees
    private static final double MIN_POWER = 0.05; // Minimum power to overcome friction
    private static final double MAX_POWER = 0.4; // Maximum turret speed

    @Override
    public void init() {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        turretMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // Use your AprilTag pipeline
        limelight.start();

        timer.reset();
    }

    @Override
    public void loop() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            // Get horizontal offset from center (tx)
            double tx = result.getTx();

            // Calculate time since last update
            double dt = timer.seconds();
            timer.reset();

            // Calculate error
            double error = targetX - tx; // Negative tx means target is left

            // PID calculations
            integral += error * dt;

            // Anti-windup: prevent integral from getting too large
            integral = Math.max(-50, Math.min(50, integral));

            double derivative = (dt > 0) ? (error - lastError) / dt : 0;

            // Calculate PID output
            double pidOutput =
                (kP * error) + (kI * integral) + (kD * derivative);

            // Apply deadband - stop if close enough
            if (Math.abs(error) < POSITION_TOLERANCE) {
                pidOutput = 0;
                integral = 0; // Reset integral when on target
            }

            // Apply minimum power if not zero (overcome static friction)
            if (pidOutput != 0) {
                if (Math.abs(pidOutput) < MIN_POWER) {
                    pidOutput = MIN_POWER * Math.signum(pidOutput);
                }
            }

            // Clamp output to max power
            double motorPower = Math.max(
                -MAX_POWER,
                Math.min(MAX_POWER, pidOutput)
            );

            turretMotor.setPower(motorPower);

            lastError = error;

            // Telemetry for tuning
            telemetry.addData("Target Visible", "Yes");
            telemetry.addData("TX (degrees)", "%.2f", tx);
            telemetry.addData("Error", "%.2f", error);
            telemetry.addData("PID Output", "%.3f", pidOutput);
            telemetry.addData("Motor Power", "%.3f", motorPower);
            telemetry.addData(
                "Tag ID",
                result.getFiducialResults().get(0).getFiducialId()
            );
        } else {
            // No target found - stop and reset
            turretMotor.setPower(0);
            integral = 0;
            lastError = 0;

            telemetry.addData("Target Visible", "No");
            telemetry.addData("Status", "Searching...");
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        turretMotor.setPower(0);
        limelight.stop();
    }
}

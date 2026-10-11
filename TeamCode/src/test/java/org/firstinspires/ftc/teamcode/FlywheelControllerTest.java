package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import java.lang.reflect.Proxy;
import org.junit.Test;
import static org.junit.Assert.*;

public class FlywheelControllerTest {
    private static class MotorState {
        double power = 1;
        double velocity;
        DcMotor.RunMode mode;
        DcMotor.ZeroPowerBehavior zeroPowerBehavior;
        PIDFCoefficients pidf;
        boolean directionChanged;

        DcMotorEx motor() {
            return (DcMotorEx) Proxy.newProxyInstance(DcMotorEx.class.getClassLoader(),
                    new Class<?>[]{DcMotorEx.class}, (proxy, method, args) -> {
                        switch (method.getName()) {
                            case "setPower": power = (double) args[0]; break;
                            case "setVelocity": velocity = (double) args[0]; break;
                            case "getVelocity": return velocity;
                            case "setMode": mode = (DcMotor.RunMode) args[0]; break;
                            case "setZeroPowerBehavior": zeroPowerBehavior = (DcMotor.ZeroPowerBehavior) args[0]; break;
                            case "setPIDFCoefficients": pidf = (PIDFCoefficients) args[1]; break;
                            case "setDirection": directionChanged = true; break;
                        }
                        return null;
                    });
        }
    }

    @Test
    public void controlsEncoderVelocityWithLastSeasonsCalibrationAndCoastsWhenOff() {
        MotorState state = new MotorState();
        FlywheelController controller = new FlywheelController(state.motor());
        assertEquals(DcMotor.RunMode.RUN_USING_ENCODER, state.mode);
        assertEquals(DcMotor.ZeroPowerBehavior.FLOAT, state.zeroPowerBehavior);
        assertFalse(state.directionChanged);
        assertEquals(9, state.pidf.p, 0);
        assertEquals(0.5, state.pidf.i, 0);
        assertEquals(15, state.pidf.f, 0);
        controller.setTargetRPM(3000);
        assertEquals(2600, state.velocity, 1e-9);
        assertEquals(3000, controller.getTargetRPM(), 1e-9);
        state.velocity = 2470;
        assertEquals(2850, controller.getCurrentRPM(), 1e-9);
        state.power = 0.8;
        controller.setTargetRPM(0);
        assertEquals(0, state.power, 0);
        assertEquals(0, controller.getTargetRPM(), 0);
        assertEquals(2470, state.velocity, 0); // Off cuts power instead of commanding a braking velocity.
    }

    @Test
    public void clampsSpeedAndRejectsInvalidTargetsBeforeSendingMotorCommands() {
        MotorState state = new MotorState();
        FlywheelController controller = new FlywheelController(state.motor());
        controller.setTargetRPM(9000);
        assertEquals(6000, controller.getTargetRPM(), 0);
        assertEquals(5200, state.velocity, 0);
        controller.setTargetRPM(-100);
        assertEquals(0, controller.getTargetRPM(), 0);
        assertThrows(IllegalArgumentException.class, () -> controller.setTargetRPM(Double.NaN));
        assertThrows(IllegalArgumentException.class, () -> controller.setTargetRPM(Double.POSITIVE_INFINITY));
    }
}

package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.Tests;
import org.firstinspires.ftc.teamcode.pedro.procedures.TwoWheelTuner;

public class Tuning {
    @Tuner
    public static Procedure mecanum() {
        return new MecanumTuner();
    }

    @Tuner
    public static Procedure twoWheel() {
        return new TwoWheelTuner();
    }

    @Tuner
    public static Procedure foresight() {
        return new ForesightTuner(Constants::createLocalizer, Constants::createDrivetrain);
    }

    @Tuner
    public static Procedure tests() {
        return new Tests(Constants::createDrivetrain, Constants::createLocalizer, Constants::createAlgorithm);
    }
}

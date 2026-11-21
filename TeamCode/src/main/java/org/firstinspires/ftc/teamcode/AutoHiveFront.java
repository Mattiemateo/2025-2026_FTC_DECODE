package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/** Start at the upward CELL: leave, shoot the preloads, park. See BiobuzzAuto. */
@Autonomous(name = "BIOBUZZ Auto 1: Hive Front", group = "BIOBUZZ", preselectTeleOp = "BIOBUZZ")
public class AutoHiveFront extends BiobuzzAuto {
    @Override
    Plan plan() {
        return Plan.HIVE_FRONT;
    }
}

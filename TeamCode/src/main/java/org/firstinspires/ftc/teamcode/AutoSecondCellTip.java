package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/** Start at the downward CELL, shoot when the partner tips it; otherwise shoot the original upward CELL. See BiobuzzAuto. */
@Autonomous(name = "BIOBUZZ Auto 2: Second Cell Fallback TIP", group = "BIOBUZZ", preselectTeleOp = "BIOBUZZ")
public class AutoSecondCellTip extends BiobuzzAuto {
    @Override
    Plan plan() {
        return Plan.SECOND_CELL_TIP;
    }
}

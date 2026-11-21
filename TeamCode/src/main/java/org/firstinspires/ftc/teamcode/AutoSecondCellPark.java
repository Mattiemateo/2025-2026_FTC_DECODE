package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/** Start at the downward CELL, shoot when the partner tips it; otherwise just park. See BiobuzzAuto. */
@Autonomous(name = "BIOBUZZ Auto 3: Second Cell Fallback PARK", group = "BIOBUZZ", preselectTeleOp = "BIOBUZZ")
public class AutoSecondCellPark extends BiobuzzAuto {
    @Override
    Plan plan() {
        return Plan.SECOND_CELL_PARK;
    }
}

package org.firstinspires.ftc.teamcode.BadAutonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.teamcode.CypherAutoMethods;

@Disabled
@Autonomous(name = "blue park only", group = "Auto")
public class bluepark extends CypherAutoMethods {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        initEverything();

        waitForStart();
        testAutoMove(0, -12);

    }
}

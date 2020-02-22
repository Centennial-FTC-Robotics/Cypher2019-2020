package org.firstinspires.ftc.teamcode.BadAutonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.teamcode.CypherAutoMethods;

@Autonomous(name = "Red Loading Bottom Park", group = "Red Loading Auto")
@Disabled
public class RedLoadingBottomPark extends CypherAutoMethods {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        initEverything();
        waitForStart();
            /* Assuming the foundation claws are facing away from the foundation,
            the robot needs to move 12 to the right and 0 forwards/backwards since
            it needs to park bottom in this path
             */

        testAutoMove(0, -12);

    }
}
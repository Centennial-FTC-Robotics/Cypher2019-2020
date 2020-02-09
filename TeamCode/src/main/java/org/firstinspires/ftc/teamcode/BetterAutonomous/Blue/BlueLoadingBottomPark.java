package org.firstinspires.ftc.teamcode.BetterAutonomous.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CypherAutoMethods;

@Autonomous(name = "Blue Loading Bottom Park", group = "Blue Loading Auto")
public class BlueLoadingBottomPark extends CypherAutoMethods {
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        initEverything();
        waitForStart();
            /* Assuming the foundation claws are facing away from the foundation,
            the robot needs to move 12 to the left and 0 forwards/backwards since
            it needs to park bottom in this path
             */

        testAutoMove(0, -12);

    }
}

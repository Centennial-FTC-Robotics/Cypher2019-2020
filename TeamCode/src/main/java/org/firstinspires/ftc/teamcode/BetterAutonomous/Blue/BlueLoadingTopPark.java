package org.firstinspires.ftc.teamcode.BetterAutonomous.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.CypherAutoMethods;
import org.firstinspires.ftc.teamcode.StopException;

@Autonomous(name = "Blue Loading Top Park", group = "Blue Loading Auto")
public class BlueLoadingTopPark extends CypherAutoMethods {
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        try {
            initEverything();
        } catch (StopException e) {
            stopEverything();
        }
        waitForStart();
        try {
            /* Assuming the foundation claws are facing away from the foundation,
            the robot needs to move 12 to the left and 20 forwards since
            it needs to park top in this path
             */

            testAutoMove(20, 0);
            testAutoMove(0, -12);
        } catch (StopException e) {
            stopEverything();
        }
    }
}

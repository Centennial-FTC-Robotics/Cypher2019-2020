package org.firstinspires.ftc.teamcode.BetterAutonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.CypherAutoMethods;
import org.firstinspires.ftc.teamcode.StopException;

@Autonomous(name = "blue parking on the opposite side towards the bottom with respect to the intial position only", group = "Auto")
public class BlueOppositeSideBottomPark {
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
            the robot needs to move 12 to the left and 0 forwards/backwards since
            it needs to park bottom in this path
             */

            testAutoMove(0, 12);
        } catch (StopException e) {
            stopEverything();
        }
    }
}

package org.firstinspires.ftc.teamcode.BetterAutonomous.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.CypherAutoMethods;

@Autonomous(name = "Blue Building Bottom Park", group = "Blue Building Auto")
public class BlueBuildingBottomPark extends CypherAutoMethods {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        initEverything();

        waitForStart();
            /* Assuming the foundation claws are facing the foundation,
            the robot needs to move 12 to the left and 0 forwards/backwards since
            it needs to park bottom in this path
             */
        testAutoMove(0, 12);

    }
}

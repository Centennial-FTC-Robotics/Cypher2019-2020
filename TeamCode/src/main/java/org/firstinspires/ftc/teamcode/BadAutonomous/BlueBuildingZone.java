package org.firstinspires.ftc.teamcode.BadAutonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.teamcode.CypherAutoMethods;
import org.firstinspires.ftc.teamcode.StopException;

@Disabled
@Autonomous(name = "Blue Team Build Zone", group = "Auto")
public class BlueBuildingZone extends CypherAutoMethods {

    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        try {
            initEverything();
        } catch (StopException e) {
            stopEverything();
        }

        waitForStart();
        currentPos.setLocation(6, 5); // set start point
        dir = 90;
        buildingAuto("blue");
    }
}

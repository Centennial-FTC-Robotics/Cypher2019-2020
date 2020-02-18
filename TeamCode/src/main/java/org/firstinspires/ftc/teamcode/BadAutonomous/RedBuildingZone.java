package org.firstinspires.ftc.teamcode.BadAutonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.teamcode.CypherAutoMethods;

@Disabled
@Autonomous(name = "Red Team Build Zone", group = "Auto")
public class RedBuildingZone extends CypherAutoMethods {

    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        initEverything();

        waitForStart();

        currentPos.setLocation(6, 5); // set start point
        dir = -90;

        buildingAuto("red");
    }
}

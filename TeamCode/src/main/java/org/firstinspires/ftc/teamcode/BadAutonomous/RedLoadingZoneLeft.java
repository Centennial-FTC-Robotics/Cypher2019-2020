package org.firstinspires.ftc.teamcode.BadAutonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.CypherAutoMethods;
import org.firstinspires.ftc.teamcode.StopException;

@Autonomous(name = "Red Team Loading Zone Left", group = "Auto")
public class RedLoadingZoneLeft extends CypherAutoMethods {

    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        try {
            initEverything();
        } catch (StopException e) {
            stopEverything();
        }

        waitForStart();

        currentPos.setLocation(6, 2);
        dir = 180;
        actualAuto(Team.RED, Side.LEFTWALL, 1);


    }
}

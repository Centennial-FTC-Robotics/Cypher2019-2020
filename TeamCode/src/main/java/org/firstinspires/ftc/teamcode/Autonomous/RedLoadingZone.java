package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CypherAutoMethods;
import org.firstinspires.ftc.teamcode.StopException;

@Autonomous(name = "Red Team Loading Zone", group = "Auto")
public class RedLoadingZone extends CypherAutoMethods {

    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        try {
            initEverything();
        } catch (StopException e) {
            stopEverything();
        }

        waitForStart();

        currentPos.setLocation(6, 2);
        dir = -90;
        loadingAuto("red", 2);


    }
}

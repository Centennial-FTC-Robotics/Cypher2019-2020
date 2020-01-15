package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CypherAutoMethods;
import org.firstinspires.ftc.teamcode.StopException;

@Autonomous(name = "red team just get foundation", group = "Auto")
public class RedMoveFoundation extends CypherAutoMethods {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        try {
            initEverything();
        } catch (StopException e) {
            stopEverything();
        }
        waitForStart();
        getFoundation(-1);


    }

}

package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CypherAutoMethods;
import org.firstinspires.ftc.teamcode.StopException;

@Autonomous(name = "red park only", group = "Auto")
public class redpark extends CypherAutoMethods {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        try {
            initEverything();
        } catch (StopException e) {
            stopEverything();
        }
        waitForStart();
        try {
            testAutoMove(20, 0);
            testAutoMove(0, 12);
        } catch (StopException e) {
            stopEverything();
        }
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Red Team Foundation", group = "Auto")
public class RedFoundation extends CypherMethods {

    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        waitForStart();
        ElapsedTime time = new ElapsedTime();
        testAutoMove(6, 0);
        turnRelative(180);
        testAutoMove(-59,17);
        moveFoundation(1);
        time.reset();
        while(time.milliseconds() < 200);
        testAutoMove(59, 0);
        moveFoundation(-1);
        time.reset();
        while(time.milliseconds() < 200);

        testAutoMove(0,129);
        skystoneFindPls(1);



    }
}

package org.firstinspires.ftc.teamcode;

public class CorrectingTest extends CypherAutoMethods {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        try {
            initEverything();
            waitForStart();
            selfCorrectStrafe(0,24);
        } catch (StopException e) {
            stopEverything();
        }

    }
}

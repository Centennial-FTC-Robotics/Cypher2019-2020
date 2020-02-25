package org.firstinspires.ftc.teamcode;

public class CorrectingTest extends CypherAutoMethods {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        initEverything();
        waitForStart();
        selfCorrectStrafe(0, 24);
    }
}

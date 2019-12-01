package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name="Encoder Test", group = "Test")
public class betterEncoder extends CypherMethods{
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        initializeIMU();

        waitForStart();
        //turnAbsolute(-90);
        testAutoMove(60, 0);
        testAutoMove(0,60);

        Thread.sleep(1000);

        moveToCenter(100, 10); //should move to the right but /shrug

    }

    private void moveToCenter(double left, double right) {
        double P = 0.02;
        double error = left - right;
        double negSpeed, posSpeed;
        double minSpeed = 0.01;
        double maxSoeed = 0.03;

        negSpeed = Range.clip(-(P*error), minSpeed, maxSoeed);
        posSpeed = Range.clip(P*error, minSpeed, maxSoeed);

        setStrafeMotors(negSpeed, posSpeed);
    }



}

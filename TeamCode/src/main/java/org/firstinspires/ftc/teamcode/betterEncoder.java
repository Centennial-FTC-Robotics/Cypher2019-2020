package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
@Autonomous(name="Encoder Test", group = "Test")
public class betterEncoder extends CypherMethods{
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        initializeIMU();

        waitForStart();
        turnAbsolute(-90);
        testAutoMove(60, 0);
    }




}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Encoder Test", group = "Test")
public class betterEncoder extends CypherMethods {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        initializeIMU();
        waitForStart();

        selfCorrectStrafe(60,60);
    }


}

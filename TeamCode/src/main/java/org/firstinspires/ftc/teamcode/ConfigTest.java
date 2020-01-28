package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@TeleOp(name = "Config Tester", group = "Test")
public class ConfigTest extends CypherMethods {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        resetEncoders();
        waitForStart();
        ElapsedTime time = new ElapsedTime();
        while (opModeIsActive()) {
            moveFoundation(0);
            while (time.seconds() < 3 && opModeIsActive()) ;
            time.reset();
            moveFoundation(1);
            while (time.seconds() < 3 && opModeIsActive()) ;
            time.reset();
        }

    }
}



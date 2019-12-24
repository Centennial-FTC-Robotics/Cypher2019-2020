package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "servoTest", group = "Test")
public class servoTest extends CypherMethods {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();
        ElapsedTime time = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        boolean toggle = true;
        while (opModeIsActive()) {
            if (time.seconds() == 5) {
                time.reset();
                toggle = !toggle;
            }
            if (toggle) {
                controlIntakeServos(1);
            } else {
                controlIntakeServos(-1);
            }

        }


    }


}

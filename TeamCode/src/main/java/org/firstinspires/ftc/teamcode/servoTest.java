package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "servoTest", group = "Test")
public class servoTest extends CypherMethods {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();

        ElapsedTime time = new ElapsedTime();
        ArmState re = ArmState.DROP;
        while (opModeIsActive()) {
            if(time.seconds() >= 2) {
                if(re.equals(ArmState.DROP)) {
                    re = ArmState.PICK;
                } else {
                    re = ArmState.DROP;
                }
            }

            switch(re) {
                case DROP:
                    grabServo(1);
                    break;
                case PICK:
                    grabServo(-1);
                    break;
            }
            telemetry.update();
        }


    }


}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name="Encoder Test", group = "Test")
public class betterEncoder extends CypherMethods{
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        initializeIMU();
        ElapsedTime time = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        waitForStart();
        //turnAbsolute(-90);

        //testAutoMove(60, 0);
        //testAutoMove(0,60);

        //Thread.sleep(1000);
        time.reset();
        do {
            moveToCenter(100, 10); //should move to the right but /shrug
        } while(time.seconds() < 20);
        //turnAbsolute(90);
        //setMotorPower(0);
        //turnRelative(-90);
    }





}

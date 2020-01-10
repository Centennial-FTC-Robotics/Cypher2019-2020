package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class mainDrive extends CypherMethods {

    @Override
    public void runOpMode() throws InterruptedException {
        final int miliTillReady = 250;
        super.runOpMode();
        waitForStart();

        ElapsedTime controller1Timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        ElapsedTime controller2Timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        resetEncoders();

        double factor = 1;
        IntakeState inState = IntakeState.STOP;
        FoundationState foundationState = FoundationState.RELEASE;
        ArmState armState = ArmState.REST;
        while (opModeIsActive()) {
            telemetry.addData("foundation state", foundationState);

            double leftPower = acutalControl(gamepad1.left_stick_x,0.45 ) * .7;
            double forwardPower = acutalControl(gamepad1.left_stick_y, 0.5) * .7;
            double rotate = acutalControl(gamepad1.right_stick_x, .3);
            boolean toggleIntake = gamepad1.a && !(gamepad1.start || gamepad2.start);
            boolean stopIntake = gamepad1.b && !(gamepad1.start || gamepad2.start);
            double vSlide = gamepad2.left_stick_y;
            double hSlide = gamepad2.right_stick_x;
            boolean arm = gamepad2.a && !(gamepad1.start || gamepad2.start);
            double swivelRight = gamepad2.right_trigger;
            double swivelLeft = -gamepad2.left_trigger;
            boolean toggleFoundation = gamepad1.y;

            telemetry.addData("vslide", getVSlidePos());
            telemetry.addData("arm", getArmPos());

            //Servo Intake Control------------------------------------------------------------------
            if (stopIntake) {
                inState = IntakeState.STOP;
            } else if (toggleIntake && controller1Timer.milliseconds() >=  miliTillReady) {
                controller1Timer.reset();
                if (inState.equals(IntakeState.OUT)) {
                    inState = IntakeState.IN;
                } else {
                    inState = IntakeState.OUT;
                }
            }

            telemetry.addData("state",inState);
            if(controller1Timer.milliseconds() > miliTillReady) {
                telemetry.addData("ready", true);
            } else {
                telemetry.addData("not ready", false);
            }


            switch (inState) {
                case IN:
                    controlIntakeServos(1);
                    break;
                case OUT:
                    controlIntakeServos(-1);
                    break;
                case STOP:
                    controlIntakeServos(0);
                    break;
            }
            //Speed Control-------------------------------------------------------------------------
           if(gamepad1.left_trigger > 0) {
               factor = 0.258;
           } else {
               factor = 0.87535463 ;
           }

            telemetry.addData("factor", factor);
            //Driving-------------------------------------------------------------------------------
            manDriveMotors(forwardPower, leftPower, rotate, factor);

            //Arm Control---------------------------------------------------------------------------

            if (arm && controller2Timer.milliseconds() >=  miliTillReady) {
                controller2Timer.reset();
                if (armState.equals(ArmState.PICK)) {
                    armState = ArmState.DROP;
                } else {
                    armState = ArmState.PICK;
                }
            }


            switch (armState) {
                case DROP:
                    grabServo(0.6);
                    break;
                case PICK:
                    grabServo(0.4834);
                    break;
            }

            controlArm(hSlide);
            controlSlides(vSlide);
            swivelServo(swivelLeft + swivelRight);

            if(toggleFoundation && controller2Timer.milliseconds() >= miliTillReady) {
                if(foundationState.equals(FoundationState.RELEASE)) {
                    foundationState = FoundationState.DRAG;
                    controlFoundation(foundationState);
                } else{
                    foundationState = FoundationState.RELEASE;
                    controlFoundation(foundationState);
                }
            }
            telemetry.update();
        }
    }
}

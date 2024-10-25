package org.firstinspires.ftc.teamcode.TeleOP;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ServoTest", group = "Testing")
@Disabled
public class servo12856 extends LinearOpMode {

    //Servo Intake1;
    //Servo Intake2;

    CRServo Intake;

   // static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
   // static final int    CYCLE_MS    =   50;     // period of each cycle
   //static final double MAX_POS     =  0.25;     // Maximum rotational position
    // static final double MIN_POS     =  0.0;

    //double  position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
    //boolean rampUp = true;

    @Override
    public void runOpMode() throws InterruptedException {
       // Intake1 = hardwareMap.get(Servo.class, "Intake1");
     //   Intake2 = hardwareMap.get(Servo.class, "Intake2");
        Intake = hardwareMap.get(CRServo.class, "Intake");

       // Intake1.setDirection(Servo.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            ServoControl();
            ContinueServo();
        }
    }
    public void ServoControl () {
        /*if (rampUp) {
            // Keep stepping up until we hit the max value.
            position += INCREMENT ;
            if (position >= MAX_POS ) {
                position = MAX_POS;
            }
        }
        else {
            // Keep stepping down until we hit the min value.
            position -= INCREMENT ;
            if (position <= MIN_POS ) {
                position = MIN_POS;
            }
        }
        if (gamepad1.left_bumper) {
            Intake1.setPosition(position);
            Intake2.setPosition(position);
        }
        if (gamepad1.right_bumper) {
            Intake2.setPosition(-position);
            Intake1.setPosition(-position);
        }*/
    }
    public void ContinueServo () {
        if (gamepad1.dpad_up) {
            Intake.setPower(0.5);
        }
        else if (gamepad1.dpad_down) {
            Intake.setPower(-0.5);
        }
        else {
            Intake.setPower(0);
        }
    }
}
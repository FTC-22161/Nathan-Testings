package org.firstinspires.ftc.teamcode.TeleOP;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(group = "TeleOp", name = "Test")
@Disabled
public class Testing extends LinearOpMode {

    DcMotorEx LeftT;
    DcMotorEx RightT;
    DcMotorEx LeftB;
    DcMotorEx RightB;

    DcMotorEx HorizontalIntake;

    Servo Intake1;
    Servo Intake2;

    CRServo Intake;

    double DriveSpeed = 0.4;

    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  0.5;     // Maximum rotational position
    static final double MIN_POS     =  0.25;

    double  position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
    boolean rampUp = true;

    @Override
    public void runOpMode() throws InterruptedException {
        LeftT = hardwareMap.get(DcMotorEx.class, "LeftT");
        RightT = hardwareMap.get(DcMotorEx.class, "RightT");
        LeftB = hardwareMap.get(DcMotorEx.class, "LeftB");
        RightB = hardwareMap.get(DcMotorEx.class, "RightB");
        Intake1 = hardwareMap.get(Servo.class, "Intake1");
        Intake2 = hardwareMap.get(Servo.class, "Intake2");
        Intake = hardwareMap.get(CRServo.class, "Intake");
        HorizontalIntake = hardwareMap.get(DcMotorEx.class, "HorizontalIntake");

        Intake1.setDirection(Servo.Direction.REVERSE);



        waitForStart();


        while (opModeIsActive() && !isStopRequested()) {
            GP1Controls();
            Servo();
            ContinueServo();
            HoriIntake();
        }
    }
    public void GP1Controls () {
        LeftT.setPower(DriveSpeed * ((gamepad1.left_stick_y) - (gamepad1.left_stick_x) - (gamepad1.right_stick_x)));
        LeftB.setPower(DriveSpeed * ((-gamepad1.left_stick_y) - (gamepad1.left_stick_x) + (gamepad1.right_stick_x)));
        RightT.setPower(DriveSpeed * ((-gamepad1.left_stick_y) - (gamepad1.left_stick_x) - (gamepad1.right_stick_x)));
        RightB.setPower(DriveSpeed * ((gamepad1.left_stick_y) - (gamepad1.left_stick_x) + (gamepad1.right_stick_x)));


        if (gamepad1.right_bumper) {
            DriveSpeed = 1;
        }
        else {
            DriveSpeed = 0.4;
        }
        /*if (gamepad1.right_bumper) {
            DriveSpeed = 1;
        }
        else if (gamepad1.left_bumper) {
            DriveSpeed = 0.4;
        }*/
    }

    //set to Servo Port 0 and 1
    public void Servo () {
        if (rampUp) {
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

        Intake1.setPosition(position * (gamepad2.right_stick_y) - (gamepad2.right_stick_y));
        Intake2.setPosition(position * (gamepad2.right_stick_y) - (gamepad2.right_stick_y));


        /*if (gamepad2.left_bumper) {
            Intake1.setPosition(position);
            Intake2.setPosition(position);
        }
        if (gamepad2.right_bumper) {
            Intake2.setPosition(-position);
            Intake1.setPosition(-position);
        }*/
    }

    //set to Servo Port 2
    public void ContinueServo () {
        Intake.setPower(0.5 * -(gamepad2.right_trigger) + (gamepad2.left_trigger));
        /*if (gamepad2.dpad_up) {
            Intake.setPower(0.5);
        }
        else if (gamepad2.dpad_down) {
            Intake.setPower(-0.5);
        }
        else {
            Intake.setPower(0);
        }*/
    }
    public void HoriIntake () {
        HorizontalIntake.setPower(0.5 * -(gamepad2.left_stick_y) + (gamepad2.left_stick_y));
    }
}
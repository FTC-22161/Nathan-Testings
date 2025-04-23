package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.robocol.Command;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.core.control.controllers.PIDFController;
import com.rowanmcalpin.nextftc.core.control.controllers.feedforward.StaticFeedforward;
import com.rowanmcalpin.nextftc.ftc.NextFTCOpMode;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.RunToPosition;

@TeleOp(name = "MiniWorlds TeleOp")
public class Mini_Worlds_TeleOp extends LinearOpMode {
    DcMotor BL, BR, FL, FR, RL, LL, RP, LP;
    Servo LightsL, LightsR, LeftS, RightS, DiffL, DiffR, Claw;
    TouchSensor ClawT, LSwitch;
    double DriveSpeed = 0.5;
    double claw_open = 0.45;
    double claw_closed = 0.19;
    boolean chicken;
    int dpad = 0;

    @Override
    public void runOpMode() {
        // Initialize hardware
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        RL = hardwareMap.get(DcMotor.class, "RL");
        LL = hardwareMap.get(DcMotor.class, "LL");
        RP = hardwareMap.get(DcMotor.class, "RP");
        LP = hardwareMap.get(DcMotor.class, "LP");
        LightsR = hardwareMap.get(Servo.class, "LightsR");
        LightsL = hardwareMap.get(Servo.class, "LightsL");
        Claw = hardwareMap.get(Servo.class, "Claw");
        LeftS = hardwareMap.get(Servo.class, "LeftS");
        RightS = hardwareMap.get(Servo.class, "RightS");
        DiffR = hardwareMap.get(Servo.class, "DiffR");
        DiffL = hardwareMap.get(Servo.class, "DiffL");
        ClawT = hardwareMap.get(TouchSensor.class, "ClawT");
        LSwitch = hardwareMap.get(TouchSensor.class, "LSwitch");
        RP.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LP.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Configure motors
        RP.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RP.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Use encoder for position control
        LP.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // LP follows RP
        RP.setDirection(DcMotor.Direction.REVERSE);
        RL.setDirection(DcMotor.Direction.REVERSE);
        DiffR.setDirection(Servo.Direction.REVERSE);
        LeftS.setDirection(Servo.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Pitch", RP.getCurrentPosition());
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Drivetrain control
            movement();


            // Claw and servo control
            if (gamepad2.dpad_up) {
                LeftS.setPosition(.15);
                RightS.setPosition(.15);

            }
            if (gamepad2.dpad_down) {
                LeftS.setPosition(0);
                RightS.setPosition(0);
                DiffL.setPosition(.15);
                DiffR.setPosition(.15);
            }
            if (gamepad2.dpad_left) {
                DiffL.setPosition(.5);
                DiffR.setPosition(.5);
            }
            // Linear actuator (RL, LL) control/
            if (gamepad2.right_bumper) {
                RL.setPower(1.5);
                LL.setPower(1.5);
            } else if (gamepad2.left_bumper) {
                RL.setPower(-1.5);
                LL.setPower(-1.5);
            } else {
                RL.setPower(0);
                LL.setPower(0);
                RP.setPower(0);
                LP.setPower(0);
                //LightsR.setPosition(.5);
                //LightsL.setPosition(.5);
            }
            if (ClawT.isPressed()) {
                Claw.setPosition(.19);
                chicken = true;
                sleep(250);
            }
            if (gamepad2.x) {
                if (chicken == true) {
                    Claw.setPosition(.45);
                    chicken = false;
                    sleep(250);
                }else if (chicken == false) {
                    Claw.setPosition(.23);
                    chicken = true;
                    sleep(250);
                }
            }


            // Pitching slides control
            /*if (gamepad2.dpad_up) {
                pitch(2856, 0.3, 5000); // Move to target position (2856 counts)
            } else if (gamepad1.dpad_up) {
                if (gamepad2.a) {
                    RP.setPower(0.5);
                    LP.setPower(0.5);
                } else if (gamepad2.b) {
                    RP.setPower(-0.5);
                    LP.setPower(-0.5);
                } else {
                    RP.setPower(.5); // Stop motors instead of opposing powers
                    LP.setPower(-.5);
                }
            } else {
                RP.setPower(.5);
                LP.setPower(-.5);
            }*/

            // Telemetry
            telemetry.addData("Pitch", RP.getCurrentPosition());
            telemetry.addData("Claw Position", Claw.getPosition());
            telemetry.addData("Touch", ClawT.isPressed());
            telemetry.addData("LSwitch", LSwitch.isPressed());
            telemetry.update();

            idle(); // Ensure other inputs are processed
        }

    }
    /*public void clawpos() {
        switch (dpad) {
            case 0:
                LeftS.setPosition(0.5);
                RightS.setPosition(0.5);
                DiffR.setPosition(0.5);
                DiffL.setPosition(0.12);
                break;
            case 1:
                LeftS.setPosition(0.32);
                RightS.setPosition(0.32);
                DiffR.setPosition(0.15);
                DiffL.setPosition(0.15);
                break;
            default:
                // No action for undefined cases
                break;
        }
    }*/

    public Command movement() {
        FL.setPower(DriveSpeed * (gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x));
        BL.setPower(DriveSpeed * (gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x));
        FR.setPower(DriveSpeed * (-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x));
        BR.setPower(DriveSpeed * (-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x));

        DriveSpeed = gamepad1.right_bumper ? 1.0 : 0.5;
        return null;
    }
    public void pitch(int targetPos, double power, int steps) {
        int currentPos = RP.getCurrentPosition();
        if (Math.abs(currentPos - targetPos) < 5 || steps <= 0) {
            RP.setPower(0);
            LP.setPower(0);
            return;
        }
        int direction = (targetPos > currentPos) ? 1 : -1;
        RP.setPower(direction * power);
        LP.setPower(direction * power);
        pitch(targetPos, power, steps - 1);

    }
    public class pitching extends Subsystem {
        // BOILERPLATE
        public final pitching INSTANCE = new pitching();
        private pitching() { }

        // USER CODE HERE
        public MotorEx RP,LP;
        public PIDFController controller = new PIDFController(0.005, 0.0, 0.0, new StaticFeedforward(0.0));
        public String name = "Pitch motors";

        public RunToPosition mid_pitch() {
            return new RunToPosition(RP, // MOTOR TO MOVE
                    500.0, // TARGET POSITION, IN TICKS
                    controller, // CONTROLLER TO IMPLEMENT
                    this); // IMPLEMENTED SUBSYSTEM
        }
        @Override
        public void initialize() {
            RP = new MotorEx("RP");
            LP = new MotorEx("LP");
        }

    }
    public class drive extends Subsystem {
        public final drive INSTANCE = new drive();
        private drive() { }
        public DcMotor BL, BR, FL, FR;
        public Command driving() {
            return movement();
        }
    }

}



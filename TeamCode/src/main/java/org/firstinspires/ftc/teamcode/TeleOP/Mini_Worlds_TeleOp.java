package org.firstinspires.ftc.teamcode.Teleop;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "Michiana Teleop with the chicke strips")
public class Michiana_Teleop extends LinearOpMode {
    public DcMotorEx FL, FR, BL, BR,LL,RL,RP,LP;
    private TouchSensor ClawT;
    public TouchSensor Lswitch;
    public Servo Sweep, DiffR, DiffL, LightL, LightR,Claw,RightS,LeftS;
    public VoltageSensor batteryVoltageSensor;
    double DriveSpeed = .5;
    double Claw_Open = .45;
    double Claw_Close = .9;
    boolean claw_state = true; //True is open False is closed
    private boolean isMoving = false;
    private ElapsedTime timer = new ElapsedTime();
    double FULLYCHARGEDBATTERYVOLTAGE = 14;
    public void runOpMode() throws InterruptedException {
        FL = hardwareMap.get(DcMotorEx.class, "FL");
        FR = hardwareMap.get(DcMotorEx.class, "FR");
        BL = hardwareMap.get(DcMotorEx.class, "BL");
        BR = hardwareMap.get(DcMotorEx.class, "BR");
        RL = hardwareMap.get(DcMotorEx.class, "RL");
        LL = hardwareMap.get(DcMotorEx.class, "LL");
        RP = hardwareMap.get(DcMotorEx.class, "RP");
        LP = hardwareMap.get(DcMotorEx.class, "LP");
        ClawT = hardwareMap.get(TouchSensor.class, "ClawT");
        Lswitch = hardwareMap.get(TouchSensor.class, "Lswitch");
        batteryVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        RightS = hardwareMap.get(Servo.class, "RightS");
        LeftS = hardwareMap.get(Servo.class, "LeftS");
        Claw = hardwareMap.get(Servo.class, "Claw");
        Sweep = hardwareMap.get(Servo.class, "Sweep");
        DiffR = hardwareMap.get(Servo.class, "DiffR");
        DiffL = hardwareMap.get(Servo.class, "DiffL");
        LightL = hardwareMap.get(Servo.class, "LightL");
        LightR = hardwareMap.get(Servo.class, "LightR");

        RP.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        RP.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        LP.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        LP.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        LP.setDirection(DcMotorEx.Direction.REVERSE);


        LL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        RL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        LL.setDirection(DcMotorEx.Direction.REVERSE);
        RL.setDirection(DcMotor.Direction.FORWARD);
        LeftS.setDirection(Servo.Direction.REVERSE);
        DiffR.setDirection(Servo.Direction.REVERSE);
        waitForStart();


        while (opModeIsActive() && !isStopRequested()) {
            LL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            drive();
            claw();
            manual();
            Claw_Touch();
            dpad_down();
            dpad_left();
            dpad_up();
            dpad_right();
            sweep();
            nathan();
            telemetry.addData("Hight", RL.getCurrentPosition());
            telemetry.addData("Pitch", RP.getCurrentPosition());
            telemetry.update();
//            if (!RP.isBusy() && !LP.isBusy()) {
//                brake();
//            }
        }
    }
    public void nathan() {
        float x = gamepad2.left_stick_y;

        if (gamepad2.left_stick_y < 0) {
            LL.setPower(getCompensatedPower(1));
            RL.setPower(getCompensatedPower(1));
        }else if (gamepad2.left_stick_y > 0) {
            LL.setPower(getCompensatedPower(-1));
            RL.setPower(getCompensatedPower(-1));
        }else {
            LL.setPower(0);
            RL.setPower(0);
        }
    }
    public void shoulder(double pos) {
        double compensated = getCompensatedServoPosition(pos);
        RightS.setPosition(compensated);
        LeftS.setPosition(compensated);
    }
    public void dpad_down() {
        double chicken = getCompensatedServoPosition(.375);
        double jeff = getCompensatedServoPosition(.5);
        double l = getCompensatedServoPosition(.15);
        double lp = getCompensatedServoPosition(.2);
        if(gamepad2.dpad_down) {
            shoulder(chicken);
            diffy(.2,.2);
            driveToPitchPositionRecursiveDual(0,10);
            RP.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            RP.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            LP.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            LP.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }
        if (gamepad2.right_bumper && !gamepad2.a && !gamepad2.b && !gamepad2.x) {
            Claw.setPosition(Claw_Close);
            claw_state = false;
            sleep(150);
            shoulder(jeff);
            diffy(DiffR.getPosition(), DiffL.getPosition());
            sleep(150);
            Claw_Touch();
            sleep(150);
            shoulder(chicken);
            diffy(DiffR.getPosition(),DiffL.getPosition());

        }
        if (gamepad2.dpad_down && gamepad2.a) {
            diffy(.2,.2);
        }
        if (gamepad2.dpad_down && gamepad2.b) {
            diffy(.05,.35);
        }
        if (gamepad2.dpad_down && gamepad2.x) {
            diffy(.35,.05);
        }
    }
    public void dpad_left() {
            if (gamepad2.dpad_left) {
                shoulder(0);
                diffy(.35,.35);
                driveToPositionRecursiveDual(RL,LL,0,.7);
                driveToPitchPositionRecursiveDual(700,10);
            }
        }
    public void dpad_up() {
        double chicken = getCompensatedServoPosition(.375);
        if (gamepad2.dpad_up) {
            shoulder(chicken);
            diffy(.2,.2);
            driveToPitchPositionRecursiveDual(700,10);
        }
    }
    public void dpad_right() {
        if (gamepad2.dpad_right) {
            double chicken = getCompensatedServoPosition(.375);
            shoulder(chicken);
            diffy(.2,.2);
            driveToPitchPositionRecursiveDual(250,50);
        }

    }
    public void manual() {
        // Pitch control (horizontal arm movement)
        if (gamepad1.x && !gamepad1.dpad_down && !gamepad1.dpad_up && !gamepad1.dpad_right && !gamepad1.dpad_left &&
                !gamepad2.dpad_right && !gamepad2.dpad_down && !gamepad2.dpad_up && !gamepad2.dpad_left) {
            double rpVel = RP.getVelocity();
            double lpVel = LP.getVelocity();

// Set a higher threshold to prevent jitter
            double velocityThreshold = 20; // Ticks/sec

// Apply low reverse power if the motor is still "coasting"
            double brakePower = 0.03; // Keep this small, adjust as needed

            if (Math.abs(rpVel) > velocityThreshold) {
                RP.setPower(-Math.signum(rpVel) * brakePower);
            } else {
                RP.setPower(0);
            }

            if (Math.abs(lpVel) > velocityThreshold) {
                LP.setPower(-Math.signum(lpVel) * brakePower);
            } else {
                LP.setPower(0);
            }

            RL.setPower(0);
            LL.setPower(0);
        } else if (gamepad1.x) {
            if (gamepad1.dpad_left) {
                RP.setPower(getCompensatedPower(-.9));
                LP.setPower(getCompensatedPower(-.9));
            } else if (gamepad1.dpad_right) {
                RP.setPower(getCompensatedPower(1));
                LP.setPower(getCompensatedPower(1));
            } else {
                RP.setPower(0);
                LP.setPower(0);
            }
            if (gamepad1.dpad_up) {
                RL.setPower(getCompensatedPower(0.7));
                LL.setPower(getCompensatedPower(0.7));
            } else if (gamepad1.dpad_down) {
                RL.setPower(getCompensatedPower(-0.7));
                LL.setPower(getCompensatedPower(-0.7));
            } else {
                RL.setPower(0);
                LL.setPower(0);
            }
        }
    }
    public void drive() {
        FL.setPower(getCompensatedPower(DriveSpeed * ((gamepad1.left_stick_y) - (gamepad1.left_stick_x) - (gamepad1.right_stick_x))));
        BL.setPower(getCompensatedPower(DriveSpeed * ((gamepad1.left_stick_y) + (gamepad1.left_stick_x) - (gamepad1.right_stick_x))));
        FR.setPower(getCompensatedPower(DriveSpeed * ((-gamepad1.left_stick_y) - (gamepad1.left_stick_x) - (gamepad1.right_stick_x))));
        BR.setPower(getCompensatedPower(DriveSpeed * ((-gamepad1.left_stick_y) + (gamepad1.left_stick_x) - (gamepad1.right_stick_x))));
        if (gamepad1.right_bumper) {
            DriveSpeed = 1;
        }else {
            DriveSpeed = .25;
        }
    }
    private double getCompensatedServoPosition(double basePosition) {
        double referenceVoltage = FULLYCHARGEDBATTERYVOLTAGE;
        double currentVoltage = batteryVoltageSensor.getVoltage();
        // Adjust the servo position slightly based on voltage
        double compensationFactor = referenceVoltage / currentVoltage;
        double compensatedPosition = basePosition * compensationFactor;

        return basePosition;
    }
    public double getCompensatedPower(double basePower) {
        double referenceVoltage = FULLYCHARGEDBATTERYVOLTAGE;
        double currentVoltage = batteryVoltageSensor.getVoltage();
        if (currentVoltage < 8.0) return 0; // safety cutoff
        double compensatedPower = basePower * (referenceVoltage / currentVoltage);
        return Math.max(-1.0, Math.min(1.0, compensatedPower)); // Clamp between -1 and 1
    }
    public Servo claw() {
        if (gamepad2.x && claw_state == true) {
            double compensatedPosition = getCompensatedServoPosition(Claw_Close);
            Claw.setPosition(compensatedPosition); //Close
            claw_state = false;
            sleep(150);
        }else if (gamepad2.x && claw_state == false) {
            double compensatedPosition = getCompensatedServoPosition(Claw_Open);
            Claw.setPosition(compensatedPosition); //Open
            claw_state = true;
            sleep(150);
        }
        return null;
    }
    public TouchSensor Claw_Touch() {
        if (ClawT.isPressed() && !gamepad2.x) {
            double compensatedPosition = getCompensatedServoPosition(Claw_Open);
            Claw.setPosition(compensatedPosition);
            LightL.setPosition(.5);
            LightR.setPosition(.5);
            claw_state = false;
        }else if (gamepad2.x) {
            LightL.setPosition(.37);
            LightR.setPosition(.37);
        }
        else {
            LightR.setPosition(.37);
            LightL.setPosition(.37);
        }
        return null;
    }
    public void driveToPositionRecursiveDual(@NonNull DcMotor RL, @NonNull DcMotor LL, int targetTicks, double basePower) {
        double power = getCompensatedPower(basePower);

        RL.setTargetPosition(targetTicks);
        LL.setTargetPosition(targetTicks);

        RL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        RL.setPower(power);
        LL.setPower(power);

        recursiveMotorCheckDual(RL, LL, targetTicks, power, 0);
    }
    private void recursiveMotorCheckDual(DcMotor RL, DcMotor LL, int target, double power, int depth) {
        if (!opModeIsActive()) return;

        boolean RLdone = !RL.isBusy();
        boolean LLdone = !LL.isBusy();

        // Base case: both motors finished or recursion too deep
        if ((RLdone && LLdone) || depth > 1000) {
            RL.setPower(0);
            LL.setPower(0);
            return;
        }

        telemetry.addData("RL Target", target);
        telemetry.addData("RL Current", RL.getCurrentPosition());
        telemetry.addData("LL Target", target);
        telemetry.addData("LL Current", LL.getCurrentPosition());
        telemetry.addData("Voltage", batteryVoltageSensor.getVoltage());
        telemetry.update();

        // Delay for tick (10ms)
        sleep(10);

        // Recursive call
        recursiveMotorCheckDual(RL, LL, target, power, depth + 1);
    }
    public void driveToPitchPositionRecursiveDual(int targetTicks, double basePower) {
        double power = getCompensatedPower(basePower);

        // Set target positions
        RP.setTargetPosition(targetTicks);
        LP.setTargetPosition(targetTicks);

        // Set run-to-position mode
        RP.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LP.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Apply power
        RP.setPower(power);
        LP.setPower(power);

        // Begin recursive monitoring
        recursiveMotorCheckDualPitch(RP, LP, targetTicks, power, 0);
    }
    private void recursiveMotorCheckDualPitch(DcMotor RP, DcMotor LP, int target, double power, int depth) {
        if (!opModeIsActive()) return;

        boolean RPdone = !RP.isBusy();
        boolean LPdone = !LP.isBusy();

        // Base case
        if ((RPdone && LPdone) || depth > 1000) {
            RP.setPower(0);
            LP.setPower(0);
            return;
        }

        telemetry.addData("RP Target", target);
        telemetry.addData("RP Current", RP.getCurrentPosition());
        telemetry.addData("LP Target", target);
        telemetry.addData("LP Current", LP.getCurrentPosition());
        telemetry.addData("Voltage", batteryVoltageSensor.getVoltage());
        telemetry.update();

        sleep(10); // delay between checks

        recursiveMotorCheckDualPitch(RP, LP, target, power, depth + 1);
    }

    public void diffy(double pos1, double pos2) {
        double poscomp1 = getCompensatedServoPosition(pos1);
        double poscomp2 = getCompensatedServoPosition(pos2);
        DiffR.setPosition(poscomp1);
        DiffL.setPosition(poscomp2);
    }
    public void sweep() {
        if (gamepad1.a) {
            Sweep.setPosition(.3);
        }else {
            Sweep.setPosition(0);
        }
    }
    public void brake() {
        double rpVel = RP.getVelocity();
        double lpVel = LP.getVelocity();

// Set a higher threshold to prevent jitter
        double velocityThreshold = 20; // Ticks/sec

// Apply low reverse power if the motor is still "coasting"
        double brakePower = 0.03; // Keep this small, adjust as needed

        if (Math.abs(rpVel) > velocityThreshold) {
            RP.setPower(-Math.signum(rpVel) * brakePower);
        } else {
            RP.setPower(0);
        }

        if (Math.abs(lpVel) > velocityThreshold) {
            LP.setPower(-Math.signum(lpVel) * brakePower);
        } else {
            LP.setPower(0);
        }

        RL.setPower(0);
        LL.setPower(0);}
}

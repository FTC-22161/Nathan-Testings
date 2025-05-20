package org.firstinspires.ftc.teamcode.example.java;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.core.command.utility.InstantCommand;
import com.rowanmcalpin.nextftc.core.command.utility.SingleFunctionCommand;
import com.rowanmcalpin.nextftc.core.command.utility.conditionals.BlockingConditionalCommand;
import com.rowanmcalpin.nextftc.core.command.utility.conditionals.PassiveConditionalCommand;
import com.rowanmcalpin.nextftc.core.control.controllers.PIDFController;
import com.rowanmcalpin.nextftc.core.control.controllers.feedforward.StaticFeedforward;
import com.rowanmcalpin.nextftc.ftc.NextFTCOpMode;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.driving.MecanumDriverControlled;
import com.rowanmcalpin.nextftc.ftc.hardware.MultipleServosToPosition;
import com.rowanmcalpin.nextftc.ftc.hardware.ServoToPosition;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.HoldPosition;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorGroup;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.ResetEncoder;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.RunToPosition;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.SetPower;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;


@TeleOp(name = "Mini_worlds_Teleop")
public class Mini_World_Teleop extends NextFTCOpMode {

    public Mini_World_Teleop() {
        super(pitch.INSTANCE,claw.INSTANCE,diff.INSTANCE,shoulder.INSTANCE,lift.INSTANCE,ClawTouch.INSTANCE);
    }
    public String frontLeftName = "FL";
    public String frontRightName = "FR";
    public String backLeftName = "BL";
    public String backRightName = "BR";
    public MotorEx FL,FR,BL,BR,RL,LL, RP,LP;
    private TouchSensor ClawT;
    public Servo Sweep,DiffR,DiffL,LightL,LightR;
    public MotorEx[] motors;
    public MotorGroup drive_train;
    public MecanumDriverControlled driverControlled;
    double DriveSpeed;
    public boolean grip = true;
    @Override
    public void onInit() {
        LL = new MotorEx("LL");
        RL = new MotorEx("RL");
        FL = new MotorEx("FL");
        BL = new MotorEx("BL");
        BR = new MotorEx("BR");
        FR = new MotorEx("FR");
        RP = new MotorEx("RP");
        LP = new MotorEx("LP");

        ClawT = OpModeData.INSTANCE.getHardwareMap().get(TouchSensor.class, "ClawT");


        // Change the motor directions to suit your robot.
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.FORWARD);
        drive_train = new MotorGroup(FL,FR,BL,BR);
        motors = new MotorEx[] {FL, FR, BL, BR};
        telemetry.addData("Status", "Touch Sensor Initialized");

        telemetry.update();


    }

    @Override
    public void onStartButtonPressed() {

        driverControlled = new MecanumDriverControlled(motors, gamepadManager.getGamepad1());
        driverControlled.invoke();


        ClawTouch.INSTANCE.checkPressed().invoke();

        gamepadManager.getGamepad1().getRightBumper().getPressedCommand();

        new ResetEncoder(LL);
        new ResetEncoder(RL);
        gamepadManager.getGamepad2().getDpadLeft().setPressedCommand(diff.INSTANCE::mid_l);
        gamepadManager.getGamepad2().getDpadLeft().setPressedCommand(diff.INSTANCE::mid_r);
        gamepadManager.getGamepad2().getDpadDown().setPressedCommand(diff.INSTANCE::grab_l);
        gamepadManager.getGamepad2().getDpadDown().setPressedCommand(diff.INSTANCE::grab_r);
        gamepadManager.getGamepad2().getDpadUp().setPressedCommand(diff.INSTANCE::drop_l);
        gamepadManager.getGamepad2().getDpadUp().setPressedCommand(diff.INSTANCE::drop_r);
        gamepadManager.getGamepad1().getX().setPressedCommand(shoulder.INSTANCE::zero);
        gamepadManager.getGamepad1().getA().setPressedCommand(shoulder.INSTANCE::not_zero);
        //gamepadManager.getGamepad1().getDpadLeft().setPressedCommand(lift.INSTANCE::Test_L);
        //gamepadManager.getGamepad1().getDpadLeft().setPressedCommand(lift.INSTANCE::Test_R);


        gamepadManager.getGamepad2().getDpadLeft().setPressedCommand(
                () -> new ParallelGroup(
                        lift.INSTANCE.Test_L(),
                        lift.INSTANCE.Test_R()
                )
        );
        gamepadManager.getGamepad2().getDpadUp().setPressedCommand(
                () -> new SequentialGroup(
                        new ParallelGroup(
                                pitch.INSTANCE.Score_L_Bar(),
                                pitch.INSTANCE.Score_R_Bar()
                        ),
                        pitch.INSTANCE.Stop()
                )
        );
        gamepadManager.getGamepad2().getDpadDown().setPressedCommand(
                () -> new ParallelGroup(
                        diff.INSTANCE.grab_l(),
                        diff.INSTANCE.grab_r()
                )
        );
        telemetry.update();
    }

    public static class ClawTouch extends Subsystem {
        // Singleton instance
        public static final ClawTouch INSTANCE = new ClawTouch();
        public ClawTouch() { }
        // Touch sensor hardware
        private TouchSensor ClawT;

        // Command to check if the sensor is pressed and update telemetry
        public Command checkPressed() {
            return new SingleFunctionCommand(() -> {
                if (ClawT.isPressed()) {
                    OpModeData.INSTANCE.getTelemetry().addData("Claw Touch Sensor", "Is Pressed");
                    OpModeData.INSTANCE.getTelemetry().update();
                    claw.INSTANCE.close();
                } else {
                    OpModeData.INSTANCE.getTelemetry().addData("Claw Touch Sensor", "Is Not Pressed");
                    OpModeData.INSTANCE.getTelemetry().update();
                }
                return true;
            });
        }

        @Override
        public void initialize() {
            ClawT = OpModeData.INSTANCE.getHardwareMap().get(TouchSensor.class, "ClawT");
        }
    }
    public static class shoulder extends Subsystem {
        public static final shoulder INSTANCE = new shoulder();
        private shoulder() {}
        Servo LeftS,RightS;
        public MultipleServosToPosition zero() {
            return new MultipleServosToPosition(
                    List.of(
                            LeftS,
                            RightS
                    ),
                    0,
                    this
            );
        }
        public MultipleServosToPosition not_zero() {
            return new MultipleServosToPosition(
                    List.of(
                            LeftS,
                            RightS
                    ),
                    .5,
                    this
            );
        }

        @Override
        public void initialize() {
            LeftS = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, "LeftS");
            RightS = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, "RightS");
            RightS.setDirection(Servo.Direction.REVERSE);
        }
    }
    public static class pitch extends Subsystem {
        public static final pitch INSTANCE = new pitch();
        private pitch() {}

        public MotorEx RP, LP;
        public MotorGroup pitching;

        public VoltageSensor voltageSensor;
        double nominalVoltage = 14.0;

        // Tuned base constants
        double baseKp = 0.001;
        double baseKd = 0.0001;
        double baseKF = 0.0;

        public PIDFController controller2;
        public PIDFController stop = new PIDFController(0.1, 0.0, 0.0001, new StaticFeedforward(0.0));

        public String LPName = "LP";
        public String RPName = "RP";

        public int target = 45;

        public RunToPosition Score_R_Bar() {
            return new RunToPosition(RP, target, controller2, this);
        }

        public RunToPosition Score_L_Bar() {
            return new RunToPosition(LP, target, controller2, this);
        }

        public HoldPosition Stop() {
            return new HoldPosition(pitching, stop, this);
        }

        @Override
        public void initialize() {
            // Motors
            RP = new MotorEx(RPName);
            LP = new MotorEx(LPName);
            LP.setDirection(DcMotorSimple.Direction.REVERSE);
            pitching = new MotorGroup(RP, LP);

            // Reset encoders
            new ResetEncoder(RP);
            new ResetEncoder(LP);

            // Voltage sensor (safe fallback)
            for (VoltageSensor sensor : OpModeData.INSTANCE.getHardwareMap().getAll(VoltageSensor.class)) {
                if (sensor.getVoltage() > 0) {
                    voltageSensor = sensor;
                    break;
                }
            }

            if (voltageSensor == null) {
                throw new RuntimeException("No valid voltage sensor found!");
            }

            // Scale PID constants based on current voltage
            double currentVoltage = voltageSensor.getVoltage();
            double scale = nominalVoltage / currentVoltage;

            double newKP = baseKp * scale;
            double newKD = baseKd * scale;
            double newKF = baseKF * scale;

            // Create scaled PIDF controller
            controller2 = new PIDFController(newKP, 0.0, newKD, new StaticFeedforward(newKF));
        }
    }
    public static class lift extends Subsystem {
        public static final lift INSTANCE = new lift();
        private lift() { }

        public MotorEx RL,LL;


        public PIDFController controller = new PIDFController(0.0025, 0.0, 0.0, new StaticFeedforward(0.0));

        public Command Test_L() {
            return new RunToPosition(LL,500,controller,this);
        }
        public Command Test_R() {
            return new RunToPosition(RL,500,controller,this);
        }

        @Override
        public void initialize() {
            RL = new MotorEx("RL");
            LL = new MotorEx("LL");
        }


    }
    public static class sweeping extends Subsystem {
        public static final sweeping INSTANCE = new sweeping();
        private sweeping() { }

        public Servo Sweep;

        public Command out() {
            return new ServoToPosition(Sweep, // SERVO TO MOVE
                    0.5, // POSITION TO MOVE TO
                    this);
        }
        public Command in() {
            return new ServoToPosition(Sweep, // SERVO TO MOVE
                    0, // POSITION TO MOVE TO
                    this);
        }
        @Override
        public void initialize() {
            Sweep = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, "Sweep");
        }
    }
    public static class diff extends Subsystem {
        public static final diff INSTANCE = new diff();
        private diff() { }

        Servo DiffR,DiffL;


        public Command mid_l() {
            return new ServoToPosition(DiffL, // SERVO TO MOVE
                    0.5, // POSITION TO MOVE TO
                    this); // IMPLEMENTED SUBSYSTEM
        }
        public Command mid_r() {
            return new ServoToPosition(DiffR, // SERVO TO MOVE
                    0.5, // POSITION TO MOVE TO
                    this); // IMPLEMENTED SUBSYSTEM
        }

        public Command grab_r() {
            return new ServoToPosition(DiffR,.9,this);
        }
        public Command grab_l() {
            return new ServoToPosition(DiffL,.1,this);
        }
        public Command drop_r() {
            return new ServoToPosition(DiffR,.18,this);
        }
        public Command drop_l() {
            return new ServoToPosition(DiffL,.83,this);
        }



        @Override
        public void initialize() {
            DiffR = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, "DiffR");
            DiffL = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, "DiffL");
        }
    }
    public static class claw extends Subsystem {
        // BOILERPLATE
        public static final claw INSTANCE = new claw();
        private claw() { }

        // USER CODE
        public Servo Claw;

        private TouchSensor ClawT;

        public String name = "Claw";

        public Command open() {
            return new ServoToPosition(Claw, // SERVO TO MOVE
                    0.9, // POSITION TO MOVE TO
                    this); // IMPLEMENTED SUBSYSTEM
        }

        public Command close() {
            return new ServoToPosition(Claw, // SERVO TO MOVE
                    0.2, // POSITION TO MOVE TO
                    this); // IMPLEMENTED SUBSYSTEM
        }

        @Override
        public void initialize() {
            Claw = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, name);
        }
    }
}



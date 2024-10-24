package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name = "Auto", group = "Autonomous")
@Disabled
public class Auto extends LinearOpMode {
    DcMotorEx LeftT;

    DcMotorEx RightT;

    DcMotorEx LeftB;

    DcMotorEx RightB;

    @Override
    public void runOpMode() throws InterruptedException {

        LeftT = hardwareMap.get(DcMotorEx.class, "LeftT");
        LeftB = hardwareMap.get(DcMotorEx.class, "LeftB");
        RightT = hardwareMap.get(DcMotorEx.class, "RightT");
        RightB = hardwareMap.get(DcMotorEx.class, "RightB");

        waitForStart();

        LeftT.setPower(0.25);
        LeftB.setPower(-0.25);
        RightT.setPower(-0.25);
        RightB.setPower(0.25);
        sleep(2000);
        LeftT.setPower(0);
        LeftB.setPower(0);
        RightT.setPower(0);
        RightB.setPower(0);

    }
}
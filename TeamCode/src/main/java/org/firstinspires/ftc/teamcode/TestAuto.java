package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Devices.DriveBase;

import java.util.concurrent.TimeUnit;

/**
 * Created by aloto10107 on 1/27/18.
 */


public class TestAuto extends LinearOpMode{

    DriveBase drive;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new DriveBase(hardwareMap);
        drive.imuINIT();
        telemetry.addData(">","" +
                "                 .eeeeeeeee\n" +
                "                .$$$$$$$$P\"\n" +
                "               .$$$$$$$$P\n" +
                "              z$$$$$$$$P\n" +
                "             z$$$$$$$$\"\n" +
                "            z$$$$$$$$\"\n" +
                "           d$$$$$$$$\"\n" +
                "          d$$$$$$$$\"\n" +
                "        .d$$$$$$$P\n" +
                "       .$$$$$$$$P\n" +
                "      .$$$$$$$$$.........\n" +
                "     .$$$$$$$$$$$$$$$$$$\"\n" +
                "    z$$$$$$$$$$$$$$$$$P\"\n" +
                "   -**********$$$$$$$P\n" +
                "             d$$$$$$\"\n" +
                "           .d$$$$$$\"\n" +
                "          .$$$$$$P\"\n" +
                "         z$$$$$$P\n" +
                "        d$$$$$$\"\n" +
                "      .d$$$$$$\"\n" +
                "     .$$$$$$$\"\n" +
                "    z$$$$$$$beeeeee\n" +
                "   d$$$$$$$$$$$$$*\n" +
                "  ^\"\"\"\"\"\"\"\"$$$$$\"\n" +
                "          d$$$*\n" +
                "         d$$$\"\n" +
                "        d$$*\n" +
                "       d$P\"\n" +
                "     .$$\"\n" +
                "    .$P\"\n" +
                "   .$\"\n" +
                "  .P\"\n" +
                " .\"     Gilo94'\n" +
                "/\"");
        telemetry.update();
        waitForStart();

//        drive.setBoth(0.5, 0.5);
//        sleep(2000);
//        drive.setBoth(0,0);

        //drive.drive(0, 2000, 0.5);
        //drive.gyroBadTurn(180);
/*
        drive.FrontandBack(1, 500);
        drive.SidetoSide(1, 500);
        drive.FrontandBack(-1, 500);
        drive. SidetoSide(-1, 500)*/;

        drive(.5,100);
    }
    public void drive(double power, double distance){
        ElapsedTime runtime = new ElapsedTime();

        if (opModeIsActive()){

            int targetLeftFront = drive.leftMotors[0].getCurrentPosition() + (int)(distance*drive.COUNTS_PER_CM);
            int targetRightFront = drive.rightMotors[0].getCurrentPosition() + (int)(distance*drive.COUNTS_PER_CM);
            int targetLeftBack = drive.leftMotors[1].getCurrentPosition() + (int)(distance*drive.COUNTS_PER_CM);
            int targetRightBack = drive.rightMotors[1].getCurrentPosition() + (int)(distance*drive.COUNTS_PER_CM);

            drive.leftMotors[0].setTargetPosition(targetLeftFront);
            drive.rightMotors[0].setTargetPosition(targetRightFront);
            drive.leftMotors[1].setTargetPosition(targetLeftBack);
            drive.rightMotors[1].setTargetPosition(targetRightBack);

            drive.leftMotors[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            drive.rightMotors[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            drive.leftMotors[1].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            drive.rightMotors[1].setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            drive.rightMotors[0].setPower(Math.abs(power));
            drive.leftMotors[1].setPower(Math.abs(power));
            drive.leftMotors[1].setPower(Math.abs(power));
            drive.leftMotors[0].setPower(Math.abs(power));

            while (opModeIsActive() && drive.rightMotors[0].isBusy() && drive.leftMotors[0].isBusy() && drive.rightMotors[1].isBusy() && drive.leftMotors[1].isBusy() && (runtime.seconds() < 5)){
                telemetry.addData("Path1",  "Running to %7d :%7d", targetLeftFront,  targetRightFront);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        drive.leftMotors[0].getCurrentPosition(),
                        drive.rightMotors[0].getCurrentPosition());
                telemetry.update();
            }
            drive.rightMotors[0].setPower(0);
            drive.leftMotors[1].setPower(0);
            drive.leftMotors[1].setPower(0);
            drive.leftMotors[0].setPower(0);


        }
    }
}

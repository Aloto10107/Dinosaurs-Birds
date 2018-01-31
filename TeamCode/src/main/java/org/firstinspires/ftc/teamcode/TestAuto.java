package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

        drive.FrontandBack(1, 1500);
    }

}

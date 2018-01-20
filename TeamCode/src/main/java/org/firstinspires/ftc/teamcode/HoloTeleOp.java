package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.Devices.DriveBase;



/**
 * Created by aloto10107 on 9/6/17.
 */

@TeleOp(name = "HoloTeleOp", group = "Driver Controlled")
public class HoloTeleOp extends OpMode {

    public DriveBase drive;
    private boolean preX;

    @Override
    public void init() {

        drive = new DriveBase(hardwareMap);
        drive.upanddown.setPosition(1);
    }

    @Override
    public void loop() {
        double leftY = gamepad1.left_stick_y*gamepad1.left_stick_y*gamepad1.left_stick_y;
        double leftX = gamepad1.left_stick_x*gamepad1.left_stick_x*gamepad1.left_stick_x;
        double rightX = (gamepad1.right_stick_x);

        if (Math.abs(gamepad1.left_stick_y) < .2) {

            leftY = 0;
        }
        if (Math.abs(gamepad1.left_stick_x) < .2) {

            leftX = 0;
        }
        if (Math.abs(gamepad1.right_stick_x) < .2) {

            rightX = 0;
        }

        drive.setMotor_bl(-leftY - leftX + rightX);
        drive.setMotor_fl(-leftY + leftX + rightX);
        drive.setMotor_br(-leftY + leftX - rightX);
        drive.setMotor_fr(-leftY - leftX - rightX);


        if (gamepad1.dpad_up) {
            drive.setMotor_bl(0.8);
            drive.setMotor_fl(0.8);
            drive.setMotor_br(0.8);
            drive.setMotor_fr(0.8);
        }
        if (gamepad1.dpad_down) {
            drive.setMotor_bl(-0.8);
            drive.setMotor_fl(-0.8);
            drive.setMotor_br(-0.8);
            drive.setMotor_fr(-0.8);
        }
        if (gamepad1.dpad_right) {
            drive.setMotor_bl(-0.8);
            drive.setMotor_fl(0.8);
            drive.setMotor_br(0.8);
            drive.setMotor_fr(-0.8);
        }
        if (gamepad1.dpad_left) {
            drive.setMotor_bl(0.8);
            drive.setMotor_fl(-0.8);
            drive.setMotor_br(-0.8);
            drive.setMotor_fr(0.8);
        }

        drive.setLift(0.9 * (gamepad2.left_stick_y));
        drive.setSidearm(0.9 * (gamepad2.right_stick_x));


        if (gamepad2.right_bumper) {
            drive.redpinch();
            drive.bluepinch();
        }
        if (!gamepad2.right_bumper) {
            drive.rednotPinch();
            drive.bluenotPinch();
        }
        if (gamepad2.left_trigger == 1) {
            drive.NoBodGot();
        }
        if (gamepad2.left_trigger != 1) {
            drive.BodGot();
        }
        if (gamepad2.x){
            drive.ReverseBodGot();
        }
        if (gamepad2.b) {
            drive.closeJaws();
        }
        if (!gamepad2.b) {
            drive.openJaws();
        }

        if (gamepad2.y) {
            drive.skillup();
        }
        if (!gamepad2.y) {
            drive.skilldown();
        }
        drive.jaws.setPosition(gamepad2.right_trigger);
        if (gamepad2.dpad_up){
            drive.jaws.setPosition(0.5);
        }
        else if (gamepad2.dpad_right){
            drive.jaws.setPosition(0.75);
        }
        else {
            drive.jaws.setPosition(1);
        }
        if (gamepad2.dpad_down){
            try {
                drive.jawsDown();
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }


     /*   if (gamepad2.dpad_down)
        {
            drive.sensordown();
        }
        if (gamepad2.dpad_up)
        {
            drive.sensorup();
        }*/
        //telemetry.addData("Heading:", String.valueOf(drive.getHeading()));
        telemetry.addData("colors:", String.valueOf(drive.getColor()[0]) + " " + String.valueOf(drive.getColor()[1]) + " " + String.valueOf(drive.getColor()[2]));
        telemetry.addData("position", drive.jaws.getPosition());
//        telemetry.addData("NormalColor", (drive.getColor()[0] - drive.getColor()[2])*1.0/drive.getColor()[0]);
    }

}




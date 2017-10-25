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
    Orientation angles;
    public ConceptVuMarkIdentification vuforia;
    private boolean preX;

    @Override
    public void init() {

        vuforia = new ConceptVuMarkIdentification();
        drive = new DriveBase(hardwareMap);
        drive.imuINIT();
    }

    @Override
    public void loop() {
        double leftY = 0.8654*gamepad1.left_stick_y;
        double leftX = 0.8654*gamepad1.left_stick_x;
        double rightX = 0.5123*gamepad1.right_stick_x;

        /*if (Math.abs(gamepad1.left_stick_y) < .1){

            leftY = 0;
        }
        if (Math.abs(gamepad1.left_stick_x) < .1){

            leftX = 0;
        }
        if (Math.abs(gamepad1.right_stick_x) < .1){

            rightX = 0;
        }*/

        drive.setMotor_bl(leftY + leftX - rightX);
        drive.setMotor_fl(leftY - leftX - rightX);
        drive.setMotor_br(leftY - leftX + rightX);
        drive.setMotor_fr(leftY + leftX + rightX);

        if (gamepad1.dpad_up){
            drive.setMotor_bl(-0.6);
            drive.setMotor_fl(-0.6);
            drive.setMotor_br(-0.6);
            drive.setMotor_fr(-0.6);
        }
        if (gamepad1.dpad_down){
            drive.setMotor_bl(0.6);
            drive.setMotor_fl(0.6);
            drive.setMotor_br(0.6);
            drive.setMotor_fr(0.6);
        }
        if (gamepad1.dpad_right){
            drive.setMotor_bl(-0.6);
            drive.setMotor_fl(0.6);
            drive.setMotor_br(0.6);
            drive.setMotor_fr(-0.6);
        }
        if (gamepad1.dpad_left){
            drive.setMotor_bl(0.6);
            drive.setMotor_fl(-0.6);
            drive.setMotor_br(-0.6);
            drive.setMotor_fr(0.6);
        }

        drive.setLift(0.9*(gamepad2.left_stick_y));
        drive.setSidearm(0.9*(gamepad2.right_stick_x));

        if (gamepad2.a){
            drive.pinch();
        }
        if (!gamepad2.a){
            drive.notPinch();
        }
        if (gamepad2.b){
            drive.closeJaws();
        }
        if (!gamepad2.b){
            drive.openJaws();
        }

        if (gamepad2.dpad_up)
        {
            drive.skillup();
        }
        if (gamepad2.dpad_down)
        {
            drive.skilldown();
        }
        if (!gamepad2.x)
        {
            preX = true;
        }
        if (gamepad2.x)
        {
            preX = false;
        }
        if (gamepad2.x && !preX)
        {
            drive.flip();
            preX = false;
        }

        //telemetry.addData("Heading:", String.valueOf(drive.getHeading()));
        telemetry.addData("colors:", String.valueOf(drive.getColor()[0]) + " " +  String.valueOf(drive.getColor()[1]) + " " + String.valueOf(drive.getColor()[2]));
    }

}



package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Devices.DriveBase;

import static com.sun.tools.javac.util.Constants.format;

/**
 * Created by aloto10107 on 10/14/17.
 */

public class BlueRightAuto extends LinearOpMode {

    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;
    DriveBase drive;
    Orientation angles;
    VectorF trans;
    public double Yerror;
    public double Xerror;
    public double Xtarget;
    double tY = 0;
    double tX = 0;
    double tZ = 0;
    double rX = 0;
    double rY = 0;
    double rZ = 0;
    long Distance = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new DriveBase(hardwareMap);
        drive.imuINIT();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AVRhcM3/////AAAAGdetb+T75E2Rt8tVnrcGqXuLsDscSqvkPx0UOvd40mxhtsQ1JoHpzZAC8DZVa0sfCpfEsmB4m4fGZkchjlAQ9Xw8GTRxvXSfmFIaog08ORICJVLzdCRAzICnf3pLy4nrT23pFZGwtwS4qau9nlNJbGpFXP56No+xcgwTzl9ZnRdZlUoUmdv12c0Ljsx2ZtoReB8MfZF7hOVe4pCwxhYFnRUUj2LPTkm62g+DSbdZXF2hlQIsoqr3jYlyShpa/CRWf4ab4NsS+OT2wY85TZd0ZlzMBGxxCFKNMPRSaMJfazDr13J472e5jA20Flq5fOKqrxzfZK19mH6jUhENsJtG99GxZkWHxcBFqmmN8WmukUzX";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTrackables.activate();
        OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
/*
        if ((pose != null) && vuMark != RelicRecoveryVuMark.UNKNOWN){
            trans = pose.getTranslation();
        }
        while(!isStarted()){
            telemetry.addData("distance", trans.get(0));
            telemetry.addData(">", "Press Play to start");
            telemetry.update();
        }*/
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
        drive.flag.setPosition(1);
        while (!isStarted()) {
            telemetry.addData("sonar",drive.getSonar());
            telemetry.update();
        }        waitForStart();

        drive.bluepinch();
        drive.redpinch();
        sleep(1000);
        drive.setLift(-.5);
        sleep(1500);
        drive.setLift(0);
        drive.upanddown.setPosition(0);
        Thread.sleep(2500);
        if((drive.getColor()[0] - drive.getColor()[2])*1.0/drive.getColor()[0] >= .5)
        {
            drive.turn(-.5,100);
            Thread.sleep(100);
            drive.upanddown.setPosition(1);
            Thread.sleep(100);
            drive.turn(.5, 100);
        }
        else if((drive.getColor()[0] - drive.getColor()[2])*1.0/drive.getColor()[0] <= 0.1)
        {
            drive.turn(.5,100);
            Thread.sleep(100);
            drive.upanddown.setPosition(1);
            Thread.sleep(100);
            drive.turn(-.5,100);
        }
        drive.upanddown.setPosition(1);
        Thread.sleep(1000);

        while (true)
        {
            drive.cry();
            telemetry.addData("Tears", drive.tears);
            telemetry.update();
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if ((vuMark != null && vuMark != RelicRecoveryVuMark.UNKNOWN) || drive.tears >= 100000)
            {
                if(vuMark == RelicRecoveryVuMark.RIGHT || vuMark == RelicRecoveryVuMark.UNKNOWN){
                    Distance = 1400;
                }
                if(vuMark == RelicRecoveryVuMark.CENTER){
                    Distance = 850;
                }
                if(vuMark == RelicRecoveryVuMark.LEFT){
                    Distance = 150;
                }
                /*drive.setBoth(.25,.25);
                sleep(2500);
                drive.setBoth(0,0);*/
                drive.FrontandBack(.25,2500,0);
                /*drive.setBoth(.5,.5);
                sleep(750);
                drive.setBoth(0,0);*/
                drive.FrontandBack(.5,750,0);
                telemetry.addData("VuMark", "%s visible", vuMark);
                telemetry.update();
                break;
            }
        }
        /*drive.setBoth(.5,.5);//off the ramp
        sleep(Distance);
        drive.setBoth(0,0);*/
        drive.FrontandBack(.5,Distance,0);
        drive.gyroTurn(90);//turn towards the cryptobox
        sleep(1000);
        /*drive.setBoth(.5,.5);//go towards correct column
        sleep(1500);
        drive.setBoth(0,0);*/
        drive.FrontandBack(.5,1500,90);
        drive.bluenotPinch();//release cubes
        drive.rednotPinch();
        drive.setBoth(-.5,-.5);
        sleep(800);
        drive.setBoth(0,0);
        sleep(900);
        drive.setBoth(.5,.5);//ram cube into correct column
        sleep(1000);
        drive.setBoth(0,0);
        sleep(900);
        drive.setBoth(-.5,-.5);//end in zone
        sleep(800);
        drive.setBoth(0,0);
        sleep(200);




       /* drive.gyroTurn(-90);
        sleep(500);
        drive.BodGot();
        sleep(100);
        drive.setLift(1);
        sleep(400);
        drive.setLift(0);
        sleep(100);
        drive.FrontandBack(1,1450,-90);
        drive.NoBodGot();
        drive.bluepinch();
        drive.redpinch();
        drive.FrontandBack(-1,500,-90);
        drive.gyroTurn(90);
        sleep(100);
        drive.setLift(-1);
        sleep(1500);
        drive.setLift(0);
        drive.FrontandBack(1,1250, 90);
        drive.bluenotPinch();
        drive.rednotPinch();
        drive.FrontandBack(-1,100,90);*/




        telemetry.addData("error", String.valueOf(drive.Gerror));
        telemetry.addData("red", drive.getColor()[0]);
        telemetry.addData("green", drive.getColor()[1]);
        telemetry.addData("blue", drive.getColor()[2]);
        telemetry.addData("Distance", drive.Derror);
        telemetry.update();
    }
}


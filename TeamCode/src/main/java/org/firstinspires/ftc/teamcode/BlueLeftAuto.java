package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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

public class BlueLeftAuto extends LinearOpMode {

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

 /*       if ((pose != null) && vuMark != RelicRecoveryVuMark.UNKNOWN){
            trans = pose.getTranslation();
        }
        while(!isStarted()){
            telemetry.addData("distance", trans.get(0));
            telemetry.addData(">", "Press Play to start lol");
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
        while (!isStarted()) {
            telemetry.addData("sonar",drive.getSonar());
            telemetry.update();
        }
        waitForStart();

        sleep(1000);
        drive.setLift(-.5);
        sleep(1500);
        drive.setLift(0);
        drive.upanddown.setPosition(0);
        Thread.sleep(2500);
        if((drive.getColor()[0] - drive.getColor()[2])*1.0/drive.getColor()[0] <= 0.1)
        {
            drive.turn(.5,100);
            Thread.sleep(100);
            drive.upanddown.setPosition(1);
            Thread.sleep(100);
            drive.turn(-.5,100);
        }
        else if((drive.getColor()[0] - drive.getColor()[2])*1.0/drive.getColor()[0] >= .5)
        {
            drive.turn(-.5,100);
            Thread.sleep(100);
            drive.upanddown.setPosition(1);
            Thread.sleep(100);
            drive.turn(.5, 100);
        }
        drive.upanddown.setPosition(1);
        Thread.sleep(1000);
//        drive.turn(-.5,500);
//        drive.gyro;Turn(0); //get fukd all of yoo try to find what i messed up
        while (true)
        {
            drive.cry();
            telemetry.addData("Tears", drive.tears);
            telemetry.update();
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if ((vuMark != null && vuMark != RelicRecoveryVuMark.UNKNOWN) || drive.tears >= 100000)
            {
                if(vuMark == RelicRecoveryVuMark.RIGHT || vuMark == RelicRecoveryVuMark.UNKNOWN){
                    Distance = 1950;
                }
                if(vuMark == RelicRecoveryVuMark.CENTER){
                    Distance = 1250;
                }
                if(vuMark == RelicRecoveryVuMark.LEFT){
                    Distance = 650;
                }
                drive.setBoth(.25,.25);
                sleep(2300);
                drive.setBoth(0,0);
                drive.setBoth(.5,.5);
                sleep(1000);
                drive.setBoth(0,0);
                telemetry.addData("VuMark", "%s visible", vuMark);
                telemetry.update();
                break;
            }
        }
//        while (true)
//        {
//            drive.cry();
//            telemetry.addData("Tears2", drive.tears);
//            telemetry.update();
//            if (pose != null)
//            {
//                break;
//            }
//        }

//        Xerror = tX - 0;
//        while (Math.abs(Xerror) >= 10) {
//            float Kp = (float) 0.005;
//            Xerror = tX - 0;
//            drive.setMotor_bl(Xerror * Kp);
//            drive.setMotor_fl(-Xerror * Kp);
//            drive.setMotor_br(-Xerror * Kp);
//            drive.setMotor_fr(Xerror * Kp);
//            if (Math.abs(Xerror) <= 4) {
//                drive.setMotor_bl(0);
//                drive.setMotor_fl(0);
//                drive.setMotor_br(0);
//                drive.setMotor_fr(0);
//                break;
//            }
//        }

        drive.setMotor_bl(-.5);
        drive.setMotor_fl(.5);
        drive.setMotor_br(.5);
        drive.setMotor_fr(-.5);
        sleep(Distance);
        drive.setBoth(0,0);
        sleep(1000);
        drive.setBoth(.5,.5);
        sleep(1000);
        drive.setBoth(0,0);
        drive.setBoth(-.5,-.5);
        sleep(700);
        drive.setBoth(0,0);
        sleep(800);
        drive.setBoth(.5,.5);
        sleep(700);
        drive.setBoth(0,0);
        sleep(800);
        drive.setBoth(-.5,-.5);
        sleep(700);
        drive.setBoth(0,0);

//       while (opModeIsActive()) {
//        /**
//         * See if any of the instances of {@link relicTemplate} are currently visible.
//         * {@link RelicRecoveryVuMark} is an enum which can have the following values:
//         * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
//         * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
//         */
//           vuMark = RelicRecoveryVuMark.from(relicTemplate);
//           if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
//
//            /* Found an instance of the template. In the actual game, you will probably
//             * loop until this condition occurs, then move on to act accordingly depending
//             * on which VuMark was visible. */
//            telemetry.addData("VuMark", "%s visible", vuMark);
//
//            /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
//             * it is perhaps unlikely that you will actually need to act on this pose information, but
//             * we illustrate it nevertheless, for completeness. */
//            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
//            telemetry.addData("Pose", format(pose));
//
//            /* We further illustrate how to decompose the pose into useful rotational and
//             * translational components */
//            if (pose != null) {
//                VectorF trans = pose.getTranslation();
//                Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
//
//                // Extract the X, Y, and Z components of the offset of the target relative to the robot
//                tX = trans.get(0);
//                tY = trans.get(1);
//                tZ = trans.get(2);
//
//                // Extract the rotational components of the target relative to the robot
//                rX = rot.firstAngle;
//                rY = rot.secondAngle;
//                rZ = rot.thirdAngle;
//
//            }
//        }
//        else {
//            telemetry.addData("VuMark", "not visible");
//        }

        telemetry.addData("error", String.valueOf(drive.Gerror));
        telemetry.addData("red", drive.getColor()[0]);
        telemetry.addData("green", drive.getColor()[1]);
        telemetry.addData("blue", drive.getColor()[2]);
        telemetry.addData("Distance", drive.Derror);
//        telemetry.addData("Z:", angles.firstAngle);
//        telemetry.addData("Y:", angles.secondAngle);
//        telemetry.addData("X:", angles.thirdAngle);
        telemetry.addData("NormalColor", (drive.getColor()[0] - drive.getColor()[2])*1.0/drive.getColor()[0]);
        telemetry.addData("Heading:", String.valueOf(drive.getHeading()));


        telemetry.update();
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


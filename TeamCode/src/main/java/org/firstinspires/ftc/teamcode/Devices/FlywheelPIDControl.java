package org.firstinspires.ftc.teamcode.Devices;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by rmmurphy on 1/26/2017.
 */

public class FlywheelPIDControl
{
    private double Ki         = 0;
    private double Kp         = 0;
    private double Kd         = 0;
    private double prevError  = 0;
    private double KiIntegral = 0;
    private double setPoint   = 0;
    private double[] maxMinLimits = {0,1};
    private double updateInterval = 0.0;
    private boolean FLY_WHEEL_SIM_EN = false;
    private ElapsedTime runtime = new ElapsedTime();
    private int simRotationCount = 0;
    private int motorDirScalar   = 1;//1 = forward, -1 = reversed
    private double deltaFlyWheelCount = 0;
    private double prevFlyWheelCount = 0;
    private double flyWheelPower = 0;
    private DcMotor Flywheel;
    private double filteredMeas = 0.0f;
    private double averageError = 0.0f;
    private double lockThreshold = 0.0f;
    private int disturbanceCount = 0;
    private int startupCount = 0;
    private boolean pidResultsReady = false;
    private double prevNanoTime = 0.0;
    private boolean initTime = true;
    private double deltaTimeSec = 0.0;

    public FlywheelPIDControl()
    {
        setKp( 0.004f);  //Proportional constant
        setKi( 0.001f);  //Integral constant
        setKd( 0.0005f); //Derivative constant
        reset();
    }

    public void setLockThreshold(double value)
    {
        lockThreshold = value;
    }

    public void setSimRotationCount(int value)
    {
        simRotationCount = value;
    }

    public double getLockThreshold()
    {
        return lockThreshold;

    }

    public void setPidMotor(DcMotor motor)
    {
        Flywheel = motor;
    }

    public double getDeltaFlyWheelCount()
    {
        return deltaFlyWheelCount;
    }

    public void setFlywheelSim(boolean value)
    {
        FLY_WHEEL_SIM_EN = value;
    }

    public boolean getFlywheelSim()
    {
        return FLY_WHEEL_SIM_EN;
    }

    public void setMotorDirScalar(int value)
    {
        motorDirScalar = value;
    }

    public int getMotorDirScalar()
    {
        return motorDirScalar;
    }

    public void setUpdateInterval( double value)
    {
        updateInterval = value;
    }

    public double getUpdateInterval()
    {
        return updateInterval;
    }

    public void setMaxMinLimits( double[] value)
    {
        maxMinLimits = value;
    }

    public double[] getMaxMinLimits()
    {
        return maxMinLimits;
    }

    public void setSetPoint(double value)
    {
        setPoint = value;
    }

    public double getPidError()
    {
        return prevError;
    }

    public void reset()
    {
        prevError  = 0.0f;
        KiIntegral = 0.0f;
        runtime.reset();
        if( FLY_WHEEL_SIM_EN)
            simRotationCount   = 0;
        prevFlyWheelCount  = 0;
        deltaFlyWheelCount = 0;
        flyWheelPower      = 0.0f;
        if( Flywheel != null)
            Flywheel.setPower(0);

        averageError = 1000.0f;
        disturbanceCount = 0;
        startupCount = 0;
        pidResultsReady = false;
        prevNanoTime = 0.0f;
        initTime = true;
    }

    public void setKp( double value)
    {
        Kp = value;
    }

    public void setKi(double value)
    {
        Ki = value;
    }

    public void setKd( double value)
    {
        Kd = value;
    }

    public double[] getCoef()
    {
        return new double[]{Kp,Ki,Kd};
    }

    public double getError()
    {
        return prevError;
    }

    private int getCurrentPosition()
    {
        if( FLY_WHEEL_SIM_EN)
            return motorDirScalar*simRotationCount;
        else
            return motorDirScalar*Flywheel.getCurrentPosition();
    }

    public boolean getPidResultsReady()
    {
        return pidResultsReady;
    }

    private void simulateEncoder()
    {
        double maxRevPerTimeStep = 400;
        double noise = (Math.random() - .5)*maxRevPerTimeStep/40;
        if( motorDirScalar == 1)
            simRotationCount = simRotationCount + (int)(flyWheelPower*maxRevPerTimeStep) + (int)noise;
        else
            simRotationCount = simRotationCount - (int)(flyWheelPower*maxRevPerTimeStep) - (int)noise;

        disturbanceCount++;
        if(disturbanceCount == 100)
        {
            disturbanceCount = 0;
            /*--------------------------------------------------------------------------------------
             * Randomly perturb the motor every so often...
             *------------------------------------------------------------------------------------*/
            double peturb = (Math.random() - .5)*maxRevPerTimeStep;
            simRotationCount = simRotationCount + (int)peturb;
        }
    }

    public boolean isLocked()
    {
        if( Math.abs(averageError) <= lockThreshold)
            return true;
        else
            return false;
    }

    public double getFilteredMeas()
    {
        return filteredMeas;
    }

    private void setAverageError(double error)
    {
        averageError = averageError*.8 + .2*error;
    }

    private double motorPIDController( double measurement, double deltaTime)
    {

        double error      = setPoint - measurement;
        double derivative = error - prevError;
        double dt = deltaTime;
        /*------------------------------------------------------------------------------------------
         * Average the error for lock detection...
         *----------------------------------------------------------------------------------------*/
        setAverageError(error);

        KiIntegral = KiIntegral + error*dt;

        filteredMeas = Ki*KiIntegral + error*Kp + derivative*Kd/dt;

        /*------------------------------------------------------------------------------------------
         * Hard limit motor speed...
         *----------------------------------------------------------------------------------------*/
        if( filteredMeas >= maxMinLimits[1])
            filteredMeas = maxMinLimits[1];
        else if( filteredMeas <= maxMinLimits[0])
            filteredMeas = maxMinLimits[0];

        prevError = error;
        return filteredMeas;
    }

    public boolean update() {
        boolean updated = false;
        int currentFlyWheelCount = 0;
        long currentNanoTime = System.nanoTime();
        if( initTime == true)
        {
            prevNanoTime = currentNanoTime;
            initTime = false;
        }

        deltaTimeSec =  (double)(currentNanoTime - prevNanoTime)  / 1000000000.0;
        /*------------------------------------------------------------------------------------------
         * Run PID loop every 'updateInterval' seconds
         *----------------------------------------------------------------------------------------*/
        if( deltaTimeSec >= updateInterval)
        {
            updated = true;
            /*--------------------------------------------------------------------------------------
             * Clear the timer...
             *------------------------------------------------------------------------------------*/
            runtime.reset();

            /*--------------------------------------------------------------------------------------
             * Get the encoder count for this interval...
             *------------------------------------------------------------------------------------*/
            currentFlyWheelCount = getCurrentPosition();
            /*--------------------------------------------------------------------------------------
             * Encoder free wheels therefore check for numerical wrapping of an integer datatype...
             *------------------------------------------------------------------------------------*/
            long deltaLong = (long) currentFlyWheelCount - (long) prevFlyWheelCount;

            deltaFlyWheelCount = (int) deltaLong;

            if( FLY_WHEEL_SIM_EN)//Simulate motor integration....
                flyWheelPower = flyWheelPower + motorPIDController(deltaFlyWheelCount,deltaTimeSec);
            else
                flyWheelPower = motorPIDController(deltaFlyWheelCount,deltaTimeSec);

            /*--------------------------------------------------------------------------------------
             * Clear the deltaCount for debugging and plotting purposes...otherwise there is a large
             * spike positive with the first sample out of reset do to the motor being off...
             *------------------------------------------------------------------------------------*/
            if (startupCount < 1) {
                deltaFlyWheelCount = currentFlyWheelCount;
                averageError = prevError;
                startupCount++;
            } else
                pidResultsReady = true;

            /*--------------------------------------------------------------------------------------
             * Limit the power applied to the motor...
             *------------------------------------------------------------------------------------*/
            if (flyWheelPower > 1.0f)
                flyWheelPower = 1.0f;
            else if (flyWheelPower < 0.0f)
                flyWheelPower = 0.0f;

            prevFlyWheelCount = currentFlyWheelCount;
            if (FLY_WHEEL_SIM_EN)
            {
                simulateEncoder();
            } else
                Flywheel.setPower(flyWheelPower);

            prevNanoTime = currentNanoTime;

        }



        return updated;
    }

}

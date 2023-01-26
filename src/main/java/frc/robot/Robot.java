package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.AHRS.SerialDataType;

import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This is a demo program providing a real-time display of navX
 * MXP values.
 *
 * In the operatorControl() method, all data from the navX sensor is retrieved
 * and output to the SmartDashboard.
 *
 * The output data values include:
 *
 * - Yaw, Pitch and Roll angles
 * - Compass Heading and 9-Axis Fused Heading (requires Magnetometer calibration)
 * - Linear Acceleration Data
 * - Motion Indicators
 * - Estimated Velocity and Displacement
 * - Quaternion Data
 * - Raw Gyro, Accelerometer and Magnetometer Data
 *
 * As well, Board Information is also retrieved; this can be useful for debugging
 * connectivity issues after initial installation of the navX MXP sensor.
 *
 */
public class Robot extends TimedRobot {
    AHRS ahrs;
    Joystick stick;

    // private static final double flywheelSpeedFast = 1.0; // fast flywheel speed
    // private static final double flywheelSpeedSlow = 0.55; // slow flywheel speed
    // private /*static final*/ double flywheelSpeed = 1.0; // Initial flywheel speed

    private static final int kFrontLeftChannel = 12;
    private static final int kRearLeftChannel = 14;
    private static final int kFrontRightChannel = 22;
    private static final int kRearRightChannel = 20;
    private static final int driverJoystickChannel = 0;
    // private static final int kFlywheelChannel = 10;// not put in yet

    // private static final int buttonShoot = 4;  // Y
    private static final int buttonTargetSeek = 7; // Back/Select

    static boolean state2 = false; // Shoot state

    private static final double slowSpeed = 0.6;
    private static final NeutralMode B_MODE = NeutralMode.Brake; // Set the talons neutralmode to brake
    private static final NeutralMode C_MODE = NeutralMode.Coast; // Set the talons neutralmode to coast

    private MecanumDrive m_robotDrive;
    private Joystick driverJoystick;

    private final Timer m_timer = new Timer(); // timer for manual shoot

    // Limelight
    private PIDController rotationController = new PIDController(0.035, 0, 0);
    private PIDController distanceController = new PIDController(0.15, 0, 0);
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tv = table.getEntry("tv");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

    // PhotonVision :: AprilTags
    // boolean cameraConnected = false;
    // PhotonCamera camera;

    // private final double CAMERA_HEIGHT_METERS = .7112;


    // DoubleSolenoid shootSol = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);


    // private WPI_TalonFX flyWheelMotor = new WPI_TalonFX(kFlywheelChannel);

    private boolean modeAuto = false;

    private float autonStartYaw;

    Thread m_visionThread;


    @Override
    public void robotInit() {
        //stick = new Joystick(0);
        // while(!cameraConnected){
        //   try {
        //     camera = new PhotonCamera("OV5647");
        //     cameraConnected = true;
        //   } catch (Exception e){
        //     System.out.println("Camera not connected, retrying");
        //   }
    
        // }

        // LiveWindow.disableAllTelemetry();
        // CameraServer.startAutomaticCapture();

        WPI_TalonSRX frontLeft = new WPI_TalonSRX(kFrontLeftChannel);//  
        WPI_TalonSRX rearLeft = new WPI_TalonSRX(kRearLeftChannel);// 
        WPI_TalonSRX frontRight = new WPI_TalonSRX(kFrontRightChannel);// 
        WPI_TalonSRX rearRight = new WPI_TalonSRX(kRearRightChannel);// 

        frontRight.setInverted(true); 
        rearRight.setInverted(true);
        frontLeft.setInverted(false);
        rearLeft.setInverted(false);

        frontLeft.setNeutralMode(B_MODE);
        rearLeft.setNeutralMode(B_MODE);
        frontRight.setNeutralMode(B_MODE);
        rearRight.setNeutralMode(B_MODE);
        // flyWheelMotor.setNeutralMode(C_MODE);


        m_robotDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);

        m_robotDrive.setDeadband(0.1);
    
        driverJoystick = new Joystick(driverJoystickChannel);


        rotationController.setTolerance(2);
        distanceController.setTolerance(2);

        // flyWheelMotor.configOpenloopRamp(0.5);

        try {
			/***********************************************************************
			 * navX-MXP:
			 * - Communication via RoboRIO MXP (SPI, I2C, TTL UART) and USB.            
			 * - See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
			 * 
			 * navX-Micro:
			 * - Communication via I2C (RoboRIO MXP or Onboard) and USB.
			 * - See http://navx-micro.kauailabs.com/guidance/selecting-an-interface.
			 * 
			 * Multiple navX-model devices on a single robot are supported.
			 ************************************************************************/
            ahrs = new AHRS(SPI.Port.kMXP);
            //ahrs = new AHRS(SerialPort.Port.kMXP, SerialDataType.kProcessedData, (byte)50);
            ahrs.enableLogging(true);
        } catch (RuntimeException ex ) {
            DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
        }
        Timer.delay(1.0);
    	//UsbCamera cam = CameraServer.getInstance().startAutomaticCapture();
    	//cam.setResolution(640, 480);        
    }

    @Override
    public void autonomousInit() 
    {
    //   l_timer.reset();
    //   l_timer.start();
    //   a_timer.reset();
    //   a_timer.start();
      m_robotDrive.setMaxOutput(slowSpeed);
      modeAuto = true;
  //    shootdistanceOK = false;
    //   shootSol.set(kForward);  // release shoot solenoid
      state2 = false; // Shoot stat
      autonStartYaw = Math.abs(ahrs.getYaw());

    }
    /**
     * Runs during autonomous mode
     */
    @Override
    public void autonomousPeriodic() 
    {
        modeAuto = true;
        Timer.delay(2.0);		//    for 2 seconds

        boolean initRotate = false;
        
        while((Math.abs(ahrs.getYaw())-autonStartYaw) < 170 && !initRotate){
            m_robotDrive.driveCartesian(0.0, 0, .8);
        }
        initRotate = true;
        // Do a 90 degree turn

        doLimelight();
    }

    /**
     * Display navX MXP Sensor Data on Smart Dashboard
     */
    @Override
    public void teleopInit() 
    {
        modeAuto = false;
        m_robotDrive.setMaxOutput(slowSpeed);
        // shootSol.set(kForward);  // release shoot solenoid
        // m_timer.start();
        // m_timer.reset();

        // while (true) {
            
            Timer.delay(0.020);		/* wait for one motor update time period (50Hz)     */
            
            boolean zero_yaw_pressed = false; //stick.getTrigger();
            if ( zero_yaw_pressed ) {
                ahrs.zeroYaw();
            }

            /* Display 6-axis Processed Angle Data                                      */
            
        // }
    }

    /**
     * Runs during test mode
     */
    public void test() {
    }

    @Override
    public void teleopPeriodic(){
        m_robotDrive.setMaxOutput(slowSpeed);
        // flywheelSpeed = flywheelSpeedFast;

        // flyWheelMotor.set(ControlMode.PercentOutput,flywheelSpeed);
        if(driverJoystick.getRawButton(buttonTargetSeek)) // use joysticks if limelight is not seeking
            doLimelight();
        else
         m_robotDrive.driveCartesian(driverJoystick.getY(), -driverJoystick.getX(),  -driverJoystick.getRawAxis(4));
      
        // shootBall(driverJoystick.getRawButton(buttonShoot));

        // SmartDashboard.putBoolean(  "IMU_Connected",        ahrs.isConnected());
        // SmartDashboard.putBoolean(  "IMU_IsCalibrating",    ahrs.isCalibrating());
        SmartDashboard.putNumber(   "IMU_Yaw",              ahrs.getYaw());
        SmartDashboard.putNumber(   "IMU_Pitch",            ahrs.getPitch());
        SmartDashboard.putNumber(   "IMU_Roll",             ahrs.getRoll());
        
        /* Display tilt-corrected, Magnetometer-based heading (requires             */
        /* magnetometer calibration to be useful)                                   */
        
        SmartDashboard.putNumber(   "IMU_CompassHeading",   ahrs.getCompassHeading());
        
        /* Display 9-axis Heading (requires magnetometer calibration to be useful)  */
        // SmartDashboard.putNumber(   "IMU_FusedHeading",     ahrs.getFusedHeading());

        /* These functions are compatible w/the WPI Gyro Class, providing a simple  */
        /* path for upgrading from the Kit-of-Parts gyro to the navx MXP            */
        
        // SmartDashboard.putNumber(   "IMU_TotalYaw",         ahrs.getAngle());
        // SmartDashboard.putNumber(   "IMU_YawRateDPS",       ahrs.getRate());

        /* Display Processed Acceleration Data (Linear Acceleration, Motion Detect) */
        
        // SmartDashboard.putNumber(   "IMU_Accel_X",          ahrs.getWorldLinearAccelX());
        // SmartDashboard.putNumber(   "IMU_Accel_Y",          ahrs.getWorldLinearAccelY());
        // SmartDashboard.putBoolean(  "IMU_IsMoving",         ahrs.isMoving());
        // SmartDashboard.putBoolean(  "IMU_IsRotating",       ahrs.isRotating());

        /* Display estimates of velocity/displacement.  Note that these values are  */
        /* not expected to be accurate enough for estimating robot position on a    */
        /* FIRST FRC Robotics Field, due to accelerometer noise and the compounding */
        /* of these errors due to single (velocity) integration and especially      */
        /* double (displacement) integration.                                       */
        
        // SmartDashboard.putNumber(   "Velocity_X",           ahrs.getVelocityX());
        // SmartDashboard.putNumber(   "Velocity_Y",           ahrs.getVelocityY());
        // SmartDashboard.putNumber(   "Displacement_X",       ahrs.getDisplacementX());
        // SmartDashboard.putNumber(   "Displacement_Y",       ahrs.getDisplacementY());
        
        /* Display Raw Gyro/Accelerometer/Magnetometer Values                       */
        /* NOTE:  These values are not normally necessary, but are made available   */
        /* for advanced users.  Before using this data, please consider whether     */
        /* the processed data (see above) will suit your needs.                     */
        
        // SmartDashboard.putNumber(   "RawGyro_X",            ahrs.getRawGyroX());
        // SmartDashboard.putNumber(   "RawGyro_Y",            ahrs.getRawGyroY());
        // SmartDashboard.putNumber(   "RawGyro_Z",            ahrs.getRawGyroZ());
        // SmartDashboard.putNumber(   "RawAccel_X",           ahrs.getRawAccelX());
        // SmartDashboard.putNumber(   "RawAccel_Y",           ahrs.getRawAccelY());
        // SmartDashboard.putNumber(   "RawAccel_Z",           ahrs.getRawAccelZ());
        // SmartDashboard.putNumber(   "RawMag_X",             ahrs.getRawMagX());
        // SmartDashboard.putNumber(   "RawMag_Y",             ahrs.getRawMagY());
        // SmartDashboard.putNumber(   "RawMag_Z",             ahrs.getRawMagZ());
        // SmartDashboard.putNumber(   "IMU_Temp_C",           ahrs.getTempC());
        // SmartDashboard.putNumber(   "IMU_Timestamp",        ahrs.getLastSensorTimestamp());
        
        /* Omnimount Yaw Axis Information                                           */
        /* For more info, see http://navx-mxp.kauailabs.com/installation/omnimount  */
        // AHRS.BoardYawAxis yaw_axis = ahrs.getBoardYawAxis();
        // SmartDashboard.putString(   "YawAxisDirection",     yaw_axis.up ? "Up" : "Down" );
        // SmartDashboard.putNumber(   "YawAxis",              yaw_axis.board_axis.getValue() );
        
        /* Sensor Board Information                                                 */
        // SmartDashboard.putString(   "FirmwareVersion",      ahrs.getFirmwareVersion());
        
        /* Quaternion Data                                                          */
        /* Quaternions are fascinating, and are the most compact representation of  */
        /* orientation data.  All of the Yaw, Pitch and Roll Values can be derived  */
        /* from the Quaternions.  If interested in motion processing, knowledge of  */
        /* Quaternions is highly recommended.                                       */
        // SmartDashboard.putNumber(   "QuaternionW",          ahrs.getQuaternionW());
        // SmartDashboard.putNumber(   "QuaternionX",          ahrs.getQuaternionX());
        // SmartDashboard.putNumber(   "QuaternionY",          ahrs.getQuaternionY());
        // SmartDashboard.putNumber(   "QuaternionZ",          ahrs.getQuaternionZ());
        
        /* Connectivity Debugging Support                                           */
        // SmartDashboard.putNumber(   "IMU_Byte_Count",       ahrs.getByteCount());
        // SmartDashboard.putNumber(   "IMU_Update_Count",     ahrs.getUpdateCount());

        /*
         * Photovision:: AprilTags
         */
        // var cameraResult = camera.getLatestResult();
        // boolean hasTargets = cameraResult.hasTargets();
        // List<PhotonTrackedTarget> targets = cameraResult.getTargets();

        // for (PhotonTrackedTarget photonTrackedTarget : targets) {
        //   double yaw = photonTrackedTarget.getYaw();
        //   int targetID = photonTrackedTarget.getFiducialId();

        //   SmartDashboard.putNumber("AprilTag " + targetID + "YAW: ", yaw);

        //   double TARGET_HEIGHT_METERS = 0;

        //   switch(targetID){
        //     case 1:
        //       TARGET_HEIGHT_METERS = 0.9620269240538481;
        //     break;
        //     case 2:
        //       TARGET_HEIGHT_METERS = 0.8350266700533401;
        //     break;
        //     case 3:
        //       TARGET_HEIGHT_METERS = 0.942975;
        //     break;
        //     case 4:
        //       TARGET_HEIGHT_METERS = 0.6731;
        //     break;

        //   }

        //   double range =
        //   PhotonUtils.calculateDistanceToTargetMeters(
        //           CAMERA_HEIGHT_METERS,
        //           TARGET_HEIGHT_METERS,
        //           .1,
        //           Units.degreesToRadians(photonTrackedTarget.getPitch()));

        // SmartDashboard.putNumber("AprilTag " + targetID  + "Range: ", range);

        // }

        /*
         * Limelight:: Apriltags
         */
        double TARGET_HEIGHT_METERS = 0;
        NetworkTableEntry validTargets = table.getEntry("json ");
        JSONParser parser = new JSONParser();
        JSONObject masterObject;
        ArrayList<Integer> displayTagArr = new ArrayList<>();
        try {
          masterObject = (JSONObject) parser.parse(validTargets.getString(""));

          JSONObject resultsObject = (JSONObject) masterObject.get("Results");
          JSONArray tagsArray = (JSONArray) resultsObject.get("Fiducial");
          for(Object tag : tagsArray){
            JSONObject jsonTag = (JSONObject) tag;
            if(!(jsonTag instanceof JSONObject)){continue;}

            int targetID = (int)jsonTag.get("fid");
            // switch(targetID){
            //   case 1:
            //     TARGET_HEIGHT_METERS = 0.9620269240538481;
            //   break;
            //   case 2:
            //     TARGET_HEIGHT_METERS = 0.8350266700533401;
            //   break;
            //   case 3:
            //     TARGET_HEIGHT_METERS = 0.942975;
            //   break;
            //   case 4:
            //     TARGET_HEIGHT_METERS = 0.6731;
            //   break;

            // }
          }
                //   SmartDashboard.putNumber("AprilTag " + targetID + "YAW: ", yaw);

        } catch (ParseException e) {
          // TODO Auto-generated catch block
          e.printStackTrace();
        }
    }

    void doLimelight()
    {
      if (!(driverJoystick.getRawButton(buttonTargetSeek) || modeAuto))
        return;

    double rotateValue = 0;
    double driveValue = 0;

    // Limelight
    double targetValid = tv.getDouble(0.0);

    if (targetValid != 1.0) // back up until target is in sight
    {
      m_robotDrive.driveCartesian(1.0, 0, 0);
      return;
    }

    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    //  double area = ta.getDouble(0.0);
  
    //SmartDashboard.putNumber("Target Valid ", targetValid);
    //SmartDashboard.putNumber("Limelight X ", x);
    //SmartDashboard.putNumber("Limelight Y ", y);
    //  SmartDashboard.putNumber("Limelight Area ", area);
  
    rotateValue = rotationController.calculate(x, 0);
    driveValue = -distanceController.calculate(y, 0);
    //SmartDashboard.putNumber("rOutput", rotateValue);
    //SmartDashboard.putNumber("dOutput", driveValue);

    //  if ((driveValue < setpointOK) && (driveValue > -setpointOK))
/*
    if (Math.abs(driveValue) < setpointOK)
      shootdistanceOK = true;
    else
      shootdistanceOK = false;
*/
    
    m_robotDrive.driveCartesian(driveValue, 0, rotateValue);
     

  /*
  if (!modeAuto) 
    {

    }
*/
  }

//   void shootBall(boolean shootTrigger)
//   {
   
//   if (shootTrigger && !state2)  // 
//     {
//     m_timer.reset();
//     shootSol.set(kReverse);
//     state2 = true;
//     }
   
//   if (m_timer.get() > 0.5)
//     shootSol.set(kForward);  // release shoot solenoid
   
//   if (!shootTrigger)
//     state2 = false;
     
//   }

  public void aprilTagCameraInit(){
    m_visionThread =
        new Thread(
            () -> {
              var camera = CameraServer.startAutomaticCapture();

              var cameraWidth = 640;
              var cameraHeight = 480;

              camera.setResolution(cameraWidth, cameraHeight);

              var cvSink = CameraServer.getVideo();
              var outputStream = CameraServer.putVideo("RioApriltags", cameraWidth, cameraHeight);

              var mat = new Mat();
              var grayMat = new Mat();

              var pt0 = new Point();
              var pt1 = new Point();
              var pt2 = new Point();
              var pt3 = new Point();
              var center = new Point();
              var red = new Scalar(0, 0, 255);
              var green = new Scalar(0, 255, 0);

              var aprilTagDetector = new AprilTagDetector();

              var config = aprilTagDetector.getConfig();
              config.quadSigma = 0.8f;
              aprilTagDetector.setConfig(config);

              var quadThreshParams = aprilTagDetector.getQuadThresholdParameters();
              quadThreshParams.minClusterPixels = 250;
              quadThreshParams.criticalAngle *= 5; // default is 10
              quadThreshParams.maxLineFitMSE *= 1.5;
              aprilTagDetector.setQuadThresholdParameters(quadThreshParams);

              aprilTagDetector.addFamily("tag16h5");

              var timer = new Timer();
              timer.start();
              var count = 0;

              while (!Thread.interrupted()) {
                if (cvSink.grabFrame(mat) == 0) {
                  outputStream.notifyError(cvSink.getError());
                  continue;
                }

                Imgproc.cvtColor(mat, grayMat, Imgproc.COLOR_RGB2GRAY);

                var results = aprilTagDetector.detect(grayMat);

                var set = new HashSet<>();

                for (var result: results) {
                  count += 1;
                  pt0.x = result.getCornerX(0);
                  pt1.x = result.getCornerX(1);
                  pt2.x = result.getCornerX(2);
                  pt3.x = result.getCornerX(3);

                  pt0.y = result.getCornerY(0);
                  pt1.y = result.getCornerY(1);
                  pt2.y = result.getCornerY(2);
                  pt3.y = result.getCornerY(3);

                  center.x = result.getCenterX();
                  center.y = result.getCenterY();

                  set.add(result.getId());

                  Imgproc.line(mat, pt0, pt1, red, 5);
                  Imgproc.line(mat, pt1, pt2, red, 5);
                  Imgproc.line(mat, pt2, pt3, red, 5);
                  Imgproc.line(mat, pt3, pt0, red, 5);

                  Imgproc.circle(mat, center, 4, green);
                  Imgproc.putText(mat, String.valueOf(result.getId()), pt2, Imgproc.FONT_HERSHEY_SIMPLEX, 2, green, 7);

                };

                for (var id : set){
                  System.out.println("Tag: " + String.valueOf(id));
                }

                if (timer.advanceIfElapsed(1.0)){
                  System.out.println("detections per second: " + String.valueOf(count));
                  count = 0;
                }

                outputStream.putFrame(mat);
              }
              aprilTagDetector.close();
            });
    m_visionThread.setDaemon(true);
    m_visionThread.start();
  }
}
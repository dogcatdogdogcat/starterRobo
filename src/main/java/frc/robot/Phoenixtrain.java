package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
// import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
// import edu.wpi.first.math.kinematics.SwerveModulePosition;
// import edu.wpi.first.wpilibj.AnalogGyro;

public class Phoenixtrain {
    // max speed for the robot
    public static final double kMaxSpeed = 3.0;

    // max rotation for robot (360 degrees)
    public static final double kMaxAngularSpeed = Math.PI;

    // calibrates where the robot knows its 'limbs' are in a 2d plane
    // with (0, 0) being the middle of the robot
    private final Translation2d m_frontRightLocation = new Translation2d(0.274, 0.274);
    private final Translation2d m_frontLeftLocation = new Translation2d(-0.274, 0.274);

    private final Translation2d m_backRightLocation = new Translation2d(0.274, -0.274);
    private final Translation2d m_backLeftLocation = new Translation2d(-0.274, -0.274);

    // driveMotorID, steeringMotorID, steeringCANcoderID
    private final PhoenixModule m_frontRight =
        new PhoenixModule(11, 12, 13);
    
    private final PhoenixModule m_frontLeft =
        new PhoenixModule(20,21, 22);

    private final PhoenixModule m_backRight =
        new PhoenixModule(14, 15, 16);
    
    private final PhoenixModule m_backLeft =
        new PhoenixModule(17, 18, 19);
    
    // private final AnalogGyro m_gyro = new AnalogGyro(0);

    private final SwerveDriveKinematics m_kinematics
        = new SwerveDriveKinematics(
            m_frontLeftLocation, m_frontRightLocation,
            m_backLeftLocation, m_backRightLocation
            );

    // private final SwerveDriveOdometry m_odometry
    //     = new SwerveDriveOdometry(
    //         m_kinematics,
    //         m_gyro.getRotation2d(),
    //         new SwerveModulePosition[] {
    //             m_frontLeft.getPosition(),
    //             m_frontRight.getPosition(),
    //             m_backLeft.getPosition(),
    //             m_backRight.getPosition()
    //           });
    
    // public Drivetrain(){
    //     m_gyro.reset();
    // }
    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed Speed of the robot in the x direction (forward).
     * @param ySpeed Speed of the robot in the y direction (sideways).
     * @param rot Angular rate of the robot.
     * @param periodSeconds Unsure of what this variable does but it seems important.
     */

    public void Drive(
        double xSpeed, double ySpeed, double rot
    ){

        var swerveModuleStates = 
            m_kinematics.toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, rot));
        
        // SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);

        System.out.println("SMS0: " + swerveModuleStates[0]);
        System.out.println("SMS1: " + swerveModuleStates[1]);
        System.out.println("SMS2: " + swerveModuleStates[2]);
        System.out.println("SMS3: " + swerveModuleStates[3]);


        // these don't do anything yet but they will tomorrow
        m_frontLeft.setDesiredState(swerveModuleStates[0], "FL");
        m_frontRight.setDesiredState(swerveModuleStates[1], "FR");

        m_backLeft.setDesiredState(swerveModuleStates[2], "BL");
        m_backRight.setDesiredState(swerveModuleStates[3], "BR");

    }

    // public void updateOdometry(){
    //     m_odometry.update(
    //         m_gyro.getRotation2d(),
    //         new SwerveModulePosition[]{
    //             m_frontLeft.getPosition(),
    //             m_frontRight.getPosition(),
    //             m_backLeft.getPosition(),
    //             m_backRight.getPosition()
    //         }
    //     );
    // }
}

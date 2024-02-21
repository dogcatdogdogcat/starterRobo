package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

public class Phoenix extends TimedRobot{
    private final XboxController m_Controller = new XboxController(0);
    private final Phoenixtrain m_Swerve = new Phoenixtrain();

    public static final double MAX_SPEED = 4.1;
    public static final double MAX_ACCEL = 4.1;
    public static final double MAX_ANGULAR_VELOCITY = 10.0;

    @Override
    public void autonomousPeriodic(){

    }

    @Override
    public void teleopPeriodic(){
        joystickDrive();
    }

    private void joystickDrive(){
        double translationVal = MathUtil.applyDeadband(m_Controller.getLeftX(), 0.2);
        double strafeVal = MathUtil.applyDeadband(m_Controller.getLeftY(), 0.2);
        double rotationVal = MathUtil.applyDeadband(m_Controller.getRightY(), 0.02);

        m_Swerve.Drive(
            new Translation2d(translationVal, strafeVal).times(MAX_SPEED),
            rotationVal * MAX_ANGULAR_VELOCITY
        );

    }
}
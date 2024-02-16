package frc.robot;
/*  
My interpretation of this file:
- To be called for the use of the robot's driving

*/

// Direct control over the robot
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

// Rotation based on a 2d plane
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

// TO BE CALLED WITHIN DRIVETRAIN.JAVA, v. important
public class SwerveModule {
	// set up for the robot ; radius of wheel and
	// resolution to encode or something
	private static final double kWheelRadius = 0.0508;
	private static final int kEncoderResolution = 4096;

	// Max velocity and acceleration for the robot
	private static final double kModuleMaxAngularVelocity
		= Drivetrain.kMaxAngularSpeed;
	private static final double kModuleMaxAngularAcceleration = 
		2 * Math.PI; // radians per second^2
		
	// motors for... driving and turning...
	private final PWMSparkMax m_driveMotor;
	private final PWMSparkMax m_turningMotor;

	// encoders for the driving and turning
	private final Encoder m_driveEncoder;
	private final Encoder m_turningEncoder;

	// create PID Controller object for controlling robot's drive systems
	// proportional, integral, derivative
	// NOTE: Has to be adjusted to fit our team's robot
	private final PIDController m_drivePIDController
		= new PIDController(1, 0, 0);

	private final ProfiledPIDController m_turningPIDController
		= new ProfiledPIDController(
			1,
			0,
			0,
			new TrapezoidProfile.Constraints(
				kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration
			)
		);
	
	// probably helps calculate feedforward commands for motor control
	private final SimpleMotorFeedforward m_driveFeedforward
		= new SimpleMotorFeedforward(1, 3);
	
	// probably helps calculate feedforward commands for turning control
	private final SimpleMotorFeedforward m_turnFeedforward
		= new SimpleMotorFeedforward(1, 0.5);

	// create swerve module to easily reference vars
	public SwerveModule(
		int driveMotorChannel,
		int turningMotorChannel,

		int driveEncoderChannelA,
		int driveEncoderChannelB,

		int turningEncoderChannelA,
		int turningEncoderChannelB
	){
		// Attach motors to their respective channels
		m_driveMotor = new PWMSparkMax(driveMotorChannel);
		m_turningMotor = new PWMSparkMax(turningMotorChannel);

		// Attach encoders to their respective channels
		m_driveEncoder = new Encoder(driveEncoderChannelA, driveEncoderChannelB);
		m_turningEncoder = new Encoder(turningEncoderChannelA, turningEncoderChannelB);

		// Dist traveled for one rotation of the wheel / encoder resolution
		m_driveEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);
		
		// Angle through an entire rotation (2 * pi ) / encoder resolution
		m_turningEncoder.setDistancePerPulse(2 * Math.PI / kEncoderResolution);
		
		// Limiting PIDController input between -pi and pi
		m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
	}

	public SwerveModuleState getState(){
		return new SwerveModuleState(
			m_driveEncoder.getRate(),
			new Rotation2d(
				m_turningEncoder.getDistance()
			)
		);
	}

	public SwerveModulePosition getPosition(){
		return new SwerveModulePosition(
			m_driveEncoder.getDistance(),
			new Rotation2d(
				m_turningEncoder.getDistance()
			)
		);
	}

	public void setDesiredState(SwerveModuleState desiredState){
		var encoderRotation = new Rotation2d(m_turningEncoder.getDistance());
		
		// avoids spinning > 90 deg
		SwerveModuleState state = SwerveModuleState.optimize(desiredState, encoderRotation);
		
		state.speedMetersPerSecond *= state.angle.minus(encoderRotation).getCos();

		final double driveOutput 
			= m_drivePIDController.calculate(m_driveEncoder.getRate(), state.speedMetersPerSecond);

		final double driveFeedforward
			= m_driveFeedforward.calculate(state.speedMetersPerSecond);

		final double turnOutput
			= m_turnFeedforward.calculate(m_turningEncoder.getDistance(), state.angle.getRadians());

		final double turnFeedforward
			= m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

		// makes da things turn
		m_driveMotor.setVoltage(driveOutput + driveFeedforward);
		m_turningMotor.setVoltage(turnOutput + turnFeedforward);
	}
}
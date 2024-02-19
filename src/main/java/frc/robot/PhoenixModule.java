// please excuse my variables ;^)
package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

@SuppressWarnings("")
public class PhoenixModule {
	// PID Controllers
	// private PIDController driveController = new PIDController(1, 1, 1);
	private PIDController steeringController = new PIDController(1, 1, 1);
		
	// DRIVING elements of the swerve module
	private CANSparkMax m_driveMotor;
	private RelativeEncoder m_driveIntegratedEncoder;

	// STEERING elements of the swerve module
	private CANSparkMax m_steeringMotor;
	private CANcoder m_steeringCANcoder;
	private RelativeEncoder m_steeringIntegratedEncoder;

	// public SwerveModuleState getState(){
	// 	return new SwerveModuleState(
	// 		m_driveIntegratedEncoder.getRate(),
	// 		new Rotation2d(
	// 			m_steeringIntegratedEncoder.getDistance()
	// 		)
	// 	);
	// }

	// public SwerveModulePosition getPosition(){
	// 	return new SwerveModulePosition(
	// 		m_driveIntegratedEncoder.getDistance(),
	// 		new Rotation2d(
	// 			m_steeringIntegratedEncoder.getDistance()
	// 		)
	// 	);
	// }

	/**
     * Method to instantiate a swerve module with motors and encoders.
     *
     * @param driveMotorID The ID found for each drive motor for the CANSparkMax.
     * @param steeringMotorID The ID found for each steering motor for the CANSparkMax.
	 * 
     * @param steeringCANcoderID The ID found for each angled absolute encoder for the swerve module.
     */
	public PhoenixModule(
		int driveMotorID,
		int steeringMotorID,

		int steeringCANcoderID
	){
			// one thing, kind of confused on the integratedEncoders for the driveMotors and
			// steering motors. I'm assuming they're important so I'll provide code here for them.
		//CANcoder
		m_steeringCANcoder = new CANcoder(steeringCANcoderID);

		// DRIVEMOTORS AND ENCODER
		m_driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
		m_driveIntegratedEncoder = m_driveMotor.getEncoder();

		// STEERING AND INTEGRATED ENCODER
		m_steeringMotor = new CANSparkMax(steeringMotorID, MotorType.kBrushless);
		m_steeringIntegratedEncoder = m_steeringMotor.getEncoder();
		m_steeringMotor.getPIDController();

		System.out.println("Drive Integrated Encoder: " + m_driveIntegratedEncoder);
		System.out.println("Steer Integrated Encoder: " + m_steeringIntegratedEncoder);
		System.out.println("STEERING MOTOR: " + m_steeringMotor);
		System.out.println("DRIVING MOTOR: " + m_driveMotor);
		System.out.println("Steering cancoder: " + m_steeringCANcoder);

		
	}

	public void setDesiredState(SwerveModuleState desiredState, String specifiedMod){
		System.out.println("DesiredState, Specified Mod: " + desiredState.speedMetersPerSecond + specifiedMod);

		m_driveMotor.set(1);
	}
}
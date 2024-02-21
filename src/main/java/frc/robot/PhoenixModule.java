// please excuse my variables ;^)
package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import com.revrobotics.SparkPIDController;

// import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class PhoenixModule {
	// PID Controllers
	// private PIDController driveController;
	private SparkPIDController steeringController;
		
	// DRIVING elements of the swerve module
	private CANSparkMax m_driveMotor;
	private RelativeEncoder m_driveIntegratedEncoder;

	// STEERING elements of the swerve module
	private CANSparkMax m_steeringMotor;
	private CANcoder m_steeringCANcoder;
	private RelativeEncoder m_steeringIntegratedEncoder;

	public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

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
		steeringController = m_steeringMotor.getPIDController();

		// PID coefficients
		kP = 0.1; 
		kI = 1e-4;
		kD = 1; 
		kIz = 0; 
		kFF = 0; 
		kMaxOutput = 1; 
		kMinOutput = -1;
	
		// set PID coefficients
		steeringController.setP(kP);
		steeringController.setI(kI);
		steeringController.setD(kD);
		steeringController.setIZone(kIz);
		steeringController.setFF(kFF);
		steeringController.setOutputRange(kMinOutput, kMaxOutput);

		System.out.println("PID Controller: " + steeringController.toString());

	}

	public void setDesiredState(SwerveModuleState desiredState, String specifiedMod){
		System.out.println("DS MPS, Module: " + desiredState.speedMetersPerSecond + specifiedMod);
		System.out.println(desiredState.angle.getDegrees());
		
		// UPDATE: ONLY FRONT RIGHT MOVES 
		// front left trying to recalibrate through rotating?

		// when commenting out steeringController.sR(), all of them turn the wheels as they should. 
		// probably some other issues of sending info to setDesiredState

		// some other issue regarding chassis speeds not being set

		// to the right right joystick 

		// THIS IS THE PROBLEM

		steeringController.setReference(desiredState.angle.getDegrees()/3, CANSparkMax.ControlType.kPosition);
		m_driveMotor.set(desiredState.speedMetersPerSecond/10);

		// desiredState.speedMetersPerSecond = 0.0;
	}
}
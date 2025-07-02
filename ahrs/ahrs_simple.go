package ahrs

import (
	"log"
	"math"
//	"fmt"
//	"github.com/skelterjohn/go.matrix"
)

const (
	minDT                      = 1e-6 // Below this time interval, don't recalculate
	maxDT                      = 10.0 // Above this time interval, re-initialize--too stale
	Pi      = math.Pi
	G       = 32.1740
	Small   = 1e-9
	Big     = 1e9
	R2D     = 180 / Pi
	D2R		= Pi / 180
	Invalid float64  = 9999999 // 2**15-1
	gyroCalDuration = 200  // Number of samples to collect (2 seconds at 100Hz)
)

type State struct {
		T float64 // Time when state last updated
}

type SimpleState struct {
	State
	T float64 // Time when state last updated
	tW                            float64                // Time of last GPS reading
	roll, pitch, heading          float64                // Fused attitude, Rad
	slipSkid                      float64                // Slip/Skid Angle, Rad
	gLoad                         float64                // G Load, G vertical
	turnRate                      float64                // turn rate, Rad/s

	// State vector (6x1): [roll, pitch, yaw, bias_x, bias_y, bias_z]
	x [6]float64
	
	// State covariance matrix (6x6)
	P [6][6]float64
	
	// Process noise covariance (6x6)
	Q [6][6]float64

	
	// Sensor characteristics
	accelNoise    float64  // Accelerometer noise variance
	gyroNoise     float64  // Gyro noise variance  
	magNoise      float64  // Magnetometer noise variance
	gpsNoise      float64  // GPS heading noise variance
	biasStability float64  // Gyro bias random walk
	accelThreshold float64     // Allow ±0.3g deviation from 1g
	magThreshold    float64     // Allow ±20% variation in mag magnitude

	// System state
	initialized bool
	valid       bool
	lastUpdate  float64


	dcm						  [3][3]float64
	needsInitialization           bool                   // Rather than computing, initialize
	logMap                        map[string]interface{} // Map only for analysis/debugging
	// Add gyro bias fields
	gyroBias                      [3]float64             // Gyro bias in deg/s (B1, B2, B3)
	needsGyroCal                  bool                   // Flag to trigger gyro calibration
	gyroCalSamples               int                    // Number of calibration samples collected
	gyroCalSum                   [3]float64             // Sum of gyro readings during calibration
	// Add magnetometer calibration
    magOffset                     [3]float64  // Hard iron offsets
    magScale                      [3]float64  // Soft iron scale factors
    magCalibrated                 bool        // Whether mag calibration is applied
}


type Measurement struct { // Order here also defines order in the matrices below
	UValid, WValid, SValid, MValid bool // Do we have valid airspeed, GPS, accel/gyro, and magnetometer readings?
	A1, A2, A3 float64 // Vector holding accelerometer readings, G, aircraft (accelerated) frame
	B1, B2, B3 float64 // Vector of gyro rates in roll, pitch, heading axes, °/s, aircraft (accelerated) frame
	M1, M2, M3 float64 // Vector of magnetometer readings, µT, aircraft (accelerated) frame y, x, and z axes
	TW, TU, T  float64 // Timestamp of GPS, airspeed and sensor readings
}













/////////////////////////////////////////////////////////////////////////
// Constructor functions
/////////////////////////////////////////////////////////////////////////

//NewSimpleAHRS returns a new Simple AHRS object.
// It is initialized with a beginning sensor orientation quaternion f0.
func NewAHRS() (s *SimpleState) {
	s = new(SimpleState)

	s.needsInitialization = true
	s.needsGyroCal = true

	// Initialize magnetometer calibration with your values M2, M1, -M3 (mx, my, mz)
    s.magOffset = [3]float64{5154, 589, 1209}
    s.magScale = [3]float64{0.000158780565259, 0.000160901045857, 0.000143802128271}
    s.magCalibrated = true

	// Initialize sensor noise parameters (tune these based on your sensors)
	s.accelNoise =     0.1     // m/s² RMS
	s.gyroNoise =      0.01    // rad/s RMS  
	s.magNoise =       0.1     // normalized units
	s.gpsNoise =       0.1     // radians
	s.biasStability =  1e-6    // rad/s/√s
	s.accelThreshold = 0.3     // Allow ±0.3g deviation from 1g
	s.magThreshold =   0.2     // Allow ±20% variation in mag magnitude

	// Initialize covariance matrices
	s.initializeCovariances()

	s.logMap = make(map[string]interface{})
	updateLogMap(s, NewMeasurement(), s.logMap)
	return
}

// NewMeasurement returns a pointer to an empty AHRS Measurement.
// Uncertainty matrix and variance accumulators are properly initialized.
func NewMeasurement() *Measurement {
	m := new(Measurement)
	return m
}



// initializeCovariances sets up the initial covariance matrices
func (s *SimpleState) initializeCovariances() {
	// Initial state uncertainty (diagonal matrix)
	// Higher uncertainty for initial attitude, lower for biases
	initialAttitudeUncertainty := 0.5  // radians
	initialBiasUncertainty := 0.1      // rad/s
	
	for i := 0; i < 6; i++ {
		for j := 0; j < 6; j++ {
			s.P[i][j] = 0
		}
	}

	// Attitude uncertainties
	s.P[0][0] = initialAttitudeUncertainty * initialAttitudeUncertainty // roll
	s.P[1][1] = initialAttitudeUncertainty * initialAttitudeUncertainty // pitch  
	s.P[2][2] = initialAttitudeUncertainty * initialAttitudeUncertainty // yaw
	
	// Bias uncertainties
	s.P[3][3] = initialBiasUncertainty * initialBiasUncertainty // bias_x
	s.P[4][4] = initialBiasUncertainty * initialBiasUncertainty // bias_y
	s.P[5][5] = initialBiasUncertainty * initialBiasUncertainty // bias_z
	
	// Process noise matrix Q (will be scaled by dt in prediction)
	for i := 0; i < 6; i++ {
		for j := 0; j < 6; j++ {
			s.Q[i][j] = 0
		}
	}
	
	// Attitude process noise (from gyro integration)
	gyroVariance := s.gyroNoise * s.gyroNoise
	s.Q[0][0] = gyroVariance // roll
	s.Q[1][1] = gyroVariance // pitch
	s.Q[2][2] = gyroVariance // yaw
	
	// Bias random walk
	biasVariance := s.biasStability * s.biasStability
	s.Q[3][3] = biasVariance // bias_x
	s.Q[4][4] = biasVariance // bias_y  
	s.Q[5][5] = biasVariance // bias_z
}











/////////////////////////////////////////////////////////////////////////
// Initialization
/////////////////////////////////////////////////////////////////////////



func (s *SimpleState) init(m *Measurement) {
	log.Printf("Initializing")
	s.needsInitialization = false

	s.T = m.T
	s.tW = m.TW

	ax, ay, az := m.A1, m.A2, m.A3
	mx, my, mz := m.M2, m.M1, -m.M3


	s.roll = math.Atan2(ay, az)
	s.pitch = math.Atan2(-ax, math.Sqrt(ay*ay + az*az))
	s.x[0] = s.roll
	s.x[1] = s.pitch


	// Apply magnetometer calibration if enabled
    if s.magCalibrated {
        mx = (mx - s.magOffset[0]) * s.magScale[0]
        my = (my - s.magOffset[1]) * s.magScale[1]
        mz = (mz - s.magOffset[2]) * s.magScale[2]
    }
	
	mmag:=math.Sqrt(mx*mx+my*my+mz*mz)

	//check if magnetometer data is invalid  (this works good!  Keeps outputs within calibration values)
	if mmag < 0.9 || mmag > 1.1 {
		mx, my, mz = 0, 0, 0
		s.needsInitialization = true 
		return
	}

	if mx!=0 && my!=0 && mz!=0 {
		//transform from body to inertial coordinates, pitch and roll only
		m1 := mx * math.Cos(s.pitch) + my * math.Sin(s.roll) * math.Sin(s.pitch) + mz * math.Cos(s.roll) * math.Sin(s.pitch);           
		m2 := my * math.Cos(s.roll) - mz * math.Sin(s.roll);
		s.heading = math.Atan2(m2, m1);
		for s.heading < 0 {
			s.heading += 2 * Pi
		}
		for s.heading >= 2*Pi {
			s.heading -= 2 * Pi
		}
	}

	s.x[2]=s.heading

	// Initialize biases to zero
	s.x[3] = 0 // bias_x
	s.x[4] = 0 // bias_y  
	s.x[5] = 0 // bias_z
	
	// Update DCM
	s.updateDCM()

	s.initialized = true
	s.valid = true
	s.lastUpdate = m.T

	log.Printf("INIT: roll %f pitch %f yaw %f", s.roll*R2D, s.pitch*R2D, s.heading*R2D)
	
	//these were fixed to use body-fixed accelerations instead of inertial accelerations
	// Initialize Slip/Skid, Rate of Turn, and GLoad.
	s.slipSkid = math.Atan2(m.A2, m.A3)*R2D
	s.turnRate = 0
	s.gLoad = m.A3 
	log.Printf("INIT: Slipskid %f turnrate %f gload %f", s.slipSkid, s.turnRate, s.gLoad)
	updateLogMap(s, m, s.logMap)

}


// calibrateGyro collects stationary gyro readings to estimate bias
func (s *SimpleState) calibrateGyro(m *Measurement) bool {
	if !s.needsGyroCal {
		return true  // Calibration already complete
	}
	
	// Check if gyro data is valid
	if !m.SValid {
		log.Printf("AHRS: Gyro calibration waiting for valid sensor data...")
		return false
	}
	
	// Accumulate gyro readings
	s.gyroCalSum[0] += m.B1
	s.gyroCalSum[1] += m.B2  
	s.gyroCalSum[2] += m.B3
	s.gyroCalSamples++
	
	// Log progress every 50 samples
	if s.gyroCalSamples%50 == 0 {
		log.Printf("AHRS: Gyro calibration progress: %d/%d samples", s.gyroCalSamples, gyroCalDuration)
	}
	
	// Check if we have enough samples
	if s.gyroCalSamples >= gyroCalDuration {
		// Calculate average bias
		s.gyroBias[0] = s.gyroCalSum[0] / float64(s.gyroCalSamples)
		s.gyroBias[1] = s.gyroCalSum[1] / float64(s.gyroCalSamples)
		s.gyroBias[2] = s.gyroCalSum[2] / float64(s.gyroCalSamples)
		
		s.needsGyroCal = false  // Mark calibration complete

		s.x[3]=s.gyroBias[0] * D2R // Convert to rad/s
		s.x[4]=s.gyroBias[1] * D2R
		s.x[5]=s.gyroBias[2] * D2R
		
		log.Printf("AHRS: Gyro calibration complete!")
		log.Printf("AHRS: Gyro biases: X=%.3f Y=%.3f Z=%.3f deg/s", 
			s.gyroBias[0], s.gyroBias[1], s.gyroBias[2])
		
		return true
	}
	
	return false  // Still calibrating
}


// ResetGyroCal triggers a new gyro calibration
func (s *SimpleState) ResetGyroCal() {
	s.needsGyroCal = true
	s.gyroCalSamples = 0
	s.gyroCalSum = [3]float64{0, 0, 0}
	s.gyroBias = [3]float64{0, 0, 0}
	log.Printf("AHRS: Gyro calibration reset - keep IMU stationary!")
}
































/////////////////////////////////////////////////////////////////////////
// Run
/////////////////////////////////////////////////////////////////////////




func (s *SimpleState) Compute(m *Measurement) {

	// First check if we need to calibrate gyro
	if s.needsGyroCal {
		gyroCalComplete := s.calibrateGyro(m)
		if !gyroCalComplete {
			return  // Don't run AHRS until gyro calibration is done
		}
	}

	if s.needsInitialization {
		s.init(m)
		log.Printf("INITIALIZATION BITCHESSSSSS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
		return
	}

	// Calculate time step
	dt := m.T - s.lastUpdate
	if dt <= 0 || dt > maxDT {
		log.Printf("Kalman DCM: Invalid dt=%.3f, reinitializing", dt)
		s.initialized = false
		s.needsInitialization = true
		return
	}
	
	// Prediction step
	s.PredictKalman(m, dt)
	
	// Measurement updates
	s.UpdateAccelerometer(m)
	s.UpdateMagnetometer(m)
	
	// Update derived quantities
	s.updateDerivedQuantities(m)
	
	s.lastUpdate = m.T
	s.valid = true
}




// Predict performs the prediction step using gyroscope measurements
func (s *SimpleState) PredictKalman(m *Measurement, dt float64) {
	if dt <= 0 || dt > maxDT {
		return
	}
	
	if !m.SValid {
		return // Need valid gyro data for prediction
	}
	
	// Extract bias-corrected gyro rates (rad/s)
	gx := (m.B1*D2R) - s.x[3] // gyro_x - bias_x
	gy := (m.B2*D2R) - s.x[4] // gyro_y - bias_y  
	gz := (m.B3*D2R) - s.x[5] // gyro_z - bias_z
	
	// Current attitude
	roll, pitch:= s.x[0], s.x[1]
	
	// Predict new attitude using gyro rates
	// This uses the small angle approximation for the rotation matrix
	// For better accuracy, could use full DCM propagation
	
	// Body-to-Euler rate transformation matrix
	sin_roll := math.Sin(roll)
	cos_roll := math.Cos(roll)
	sin_pitch := math.Sin(pitch)
	cos_pitch := math.Cos(pitch)
	tan_pitch := math.Tan(pitch)
	
	// Avoid singularity at ±90° pitch
	if math.Abs(cos_pitch) < 0.01 {
		cos_pitch = math.Copysign(0.01, cos_pitch)
	}
	
	// Euler angle rates from body rates
	roll_dot := gx + sin_roll*tan_pitch*gy + cos_roll*tan_pitch*gz
	pitch_dot := cos_roll*gy - sin_roll*gz
	yaw_dot := (sin_roll/cos_pitch)*gy + (cos_roll/cos_pitch)*gz
	
	// Update state (biases don't change in prediction)
	s.x[0] += roll_dot * dt  // roll
	s.x[1] += pitch_dot * dt // pitch
	s.x[2] += yaw_dot * dt   // yaw
	
	// Normalize angles
	s.x[2] = math.Mod(s.x[2], 2*math.Pi)
	if s.x[2] < 0 {
		s.x[2] += 2*math.Pi
	}

	// Update attitude variables for compatibility
	s.roll = s.x[0]
	s.pitch = s.x[1]
	s.heading = s.x[2]
	
	// Jacobian of state transition (F matrix)
	F := [6][6]float64{}
	
	// Identity for all states
	for i := 0; i < 6; i++ {
		F[i][i] = 1.0
	}
	
	// Derivatives of Euler rates with respect to roll and pitch
	sec_pitch := 1.0 / cos_pitch
	
	F[0][0] += dt * (cos_roll*tan_pitch*gy - sin_roll*tan_pitch*gz)
	F[0][1] += dt * (sin_roll*sec_pitch*sec_pitch*gy + cos_roll*sec_pitch*sec_pitch*gz)
	F[1][0] += dt * (-sin_roll*gy - cos_roll*gz)
	F[2][0] += dt * ((cos_roll/cos_pitch)*gy - (sin_roll/cos_pitch)*gz)
	F[2][1] += dt * ((sin_roll*sin_pitch/(cos_pitch*cos_pitch))*gy + (cos_roll*sin_pitch/(cos_pitch*cos_pitch))*gz)
	
	// Derivatives with respect to biases (negative because we subtract bias)
	F[0][3] = -dt
	F[1][4] = -dt * cos_roll
	F[1][5] = dt * sin_roll
	F[2][4] = -dt * sin_roll / cos_pitch
	F[2][5] = -dt * cos_roll / cos_pitch
	
	// Update covariance: P = F*P*F' + Q*dt
	var temp [6][6]float64
	var newP [6][6]float64
	
	// temp = F * P
	for i := 0; i < 6; i++ {
		for j := 0; j < 6; j++ {
			temp[i][j] = 0
			for k := 0; k < 6; k++ {
				temp[i][j] += F[i][k] * s.P[k][j]
			}
		}
	}
	
	// newP = temp * F'
	for i := 0; i < 6; i++ {
		for j := 0; j < 6; j++ {
			newP[i][j] = 0
			for k := 0; k < 6; k++ {
				newP[i][j] += temp[i][k] * F[j][k] // F[j][k] is F'[k][j]
			}
		}
	}
	
	// Add process noise: P = P + Q*dt
	for i := 0; i < 6; i++ {
		for j := 0; j < 6; j++ {
			s.P[i][j] = newP[i][j] + s.Q[i][j]*dt
		}
	}
	
	// Update DCM
	s.updateDCM()
}

// UpdateAccelerometer performs measurement update using accelerometer
func (s *SimpleState) UpdateAccelerometer(m *Measurement) {
	if !m.SValid {
		return
	}
	
	ax, ay, az := m.A1, m.A2, m.A3
	
	// Check if accelerometer reading is reasonable (close to 1g)
	accel_mag := math.Sqrt(ax*ax + ay*ay + az*az)
	if math.Abs(accel_mag - 1.0) > s.accelThreshold {
		return // Reject measurement - likely in acceleration
	}
	
	// Expected gravity vector in body frame from current attitude
	roll, pitch := s.x[0], s.x[1]
	
	// Predicted accelerometer readings (gravity vector rotated to body frame)
	ax_pred := -math.Sin(pitch)
	ay_pred := math.Sin(roll) * math.Cos(pitch)
	az_pred := math.Cos(roll) * math.Cos(pitch)
	
	// Innovation (measurement residual)
	innovation := [3]float64{
		ax - ax_pred,
		ay - ay_pred,  
		az - az_pred,
	}
	
	// Measurement Jacobian (H matrix) - derivatives of h(x) with respect to state
	H := [3][6]float64{}
	
	// Derivatives with respect to roll
	H[1][0] = math.Cos(roll) * math.Cos(pitch)   // day/droll
	H[2][0] = -math.Sin(roll) * math.Cos(pitch)  // daz/droll
	
	// Derivatives with respect to pitch  
	H[0][1] = -math.Cos(pitch)                    // dax/dpitch
	H[1][1] = -math.Sin(roll) * math.Sin(pitch)  // day/dpitch
	H[2][1] = -math.Cos(roll) * math.Sin(pitch)  // daz/dpitch
	
	// No dependence on yaw or biases for accelerometer
	
	// Measurement noise covariance
	R := [3][3]float64{
		{s.accelNoise * s.accelNoise, 0, 0},
		{0, s.accelNoise * s.accelNoise, 0},
		{0, 0, s.accelNoise * s.accelNoise},
	}
	
	// Convert arrays to proper slice format for kalmanUpdate
	innovation_slice := make([]float64, 3)
	H_slice := make([][]float64, 3)
	R_slice := make([][]float64, 3)
	
	for i := 0; i < 3; i++ {
		innovation_slice[i] = innovation[i]
		H_slice[i] = make([]float64, 6)
		R_slice[i] = make([]float64, 3)
		
		for j := 0; j < 6; j++ {
			H_slice[i][j] = H[i][j]
		}
		for j := 0; j < 3; j++ {
			R_slice[i][j] = R[i][j]
		}
	}
	
	// Perform Kalman update
	s.kalmanUpdate(innovation_slice, H_slice, R_slice, 3)
}

// UpdateMagnetometer performs measurement update using magnetometer
func (s *SimpleState) UpdateMagnetometer(m *Measurement) {
	if !m.MValid {
		return
	}
	
	mx, my, mz := m.M2, m.M1, -m.M3
	
	// Apply magnetometer calibration
	if s.magCalibrated {
		mx = (mx - s.magOffset[0]) * s.magScale[0]
		my = (my - s.magOffset[1]) * s.magScale[1]
		mz = (mz - s.magOffset[2]) * s.magScale[2]
	}
	
	// Check magnetometer magnitude
	mag_mag := math.Sqrt(mx*mx + my*my + mz*mz)
	if mag_mag < (1.0-s.magThreshold) || mag_mag > (1.0+s.magThreshold) {
		return // Reject measurement
	}
	
	// Normalize magnetometer reading
	mx /= mag_mag
	my /= mag_mag
	mz /= mag_mag
	
	// Current attitude
	roll, pitch := s.x[0], s.x[1] // Remove unused yaw variable
	
	// This is a simplified Jacobian - full implementation would need DCM derivatives
	// For now, just update yaw based on tilt-compensated compass
	roll_cos := math.Cos(roll)
	roll_sin := math.Sin(roll)
	pitch_cos := math.Cos(pitch)
	pitch_sin := math.Sin(pitch)
	
	mx_comp := mx*pitch_cos + my*roll_sin*pitch_sin + mz*roll_cos*pitch_sin
	my_comp := my*roll_cos - mz*roll_sin
	measured_yaw := math.Atan2(my_comp, mx_comp)
	
	// Normalize measured yaw
	for measured_yaw < 0 {
		measured_yaw += 2*math.Pi
	}
	for measured_yaw >= 2*math.Pi {
		measured_yaw -= 2*math.Pi
	}
	
	// Handle yaw wraparound
	yaw_innovation := measured_yaw - s.x[2] // Use s.x[2] directly
	if yaw_innovation > math.Pi {
		yaw_innovation -= 2*math.Pi
	} else if yaw_innovation < -math.Pi {
		yaw_innovation += 2*math.Pi
	}
	
	// Simple update for yaw only
	yaw_uncertainty := s.P[2][2] 
	mag_noise_var := s.magNoise * s.magNoise
	
	kalman_gain := yaw_uncertainty / (yaw_uncertainty + mag_noise_var)
	s.x[2] += kalman_gain * yaw_innovation
	s.P[2][2] *= (1 - kalman_gain)
	
	// Normalize yaw
	s.x[2] = math.Mod(s.x[2], 2*math.Pi)
	if s.x[2] < 0 {
		s.x[2] += 2*math.Pi
	}
	
	// Update heading variable for compatibility
	s.heading = s.x[2]
}

// kalmanUpdate performs the Kalman filter measurement update
func (s *SimpleState) kalmanUpdate(innovation []float64, H [][]float64, R [][]float64, meas_size int) {
	// S = H*P*H' + R (innovation covariance)
	var HPH [3][3]float64
	var S [3][3]float64
	
	// HPH = H * P * H'
	for i := 0; i < meas_size; i++ {
		for j := 0; j < meas_size; j++ {
			HPH[i][j] = 0
			for k := 0; k < 6; k++ {
				for l := 0; l < 6; l++ {
					HPH[i][j] += H[i][k] * s.P[k][l] * H[j][l]
				}
			}
			S[i][j] = HPH[i][j] + R[i][j]
		}
	}
	
	// Invert S (for 3x3 matrix)
	var S_inv [3][3]float64
	det := S[0][0]*(S[1][1]*S[2][2] - S[1][2]*S[2][1]) -
		  S[0][1]*(S[1][0]*S[2][2] - S[1][2]*S[2][0]) +
		  S[0][2]*(S[1][0]*S[2][1] - S[1][1]*S[2][0])
	
	if math.Abs(det) < 1e-10 {
		return // Singular matrix, skip update
	}
	
	S_inv[0][0] = (S[1][1]*S[2][2] - S[1][2]*S[2][1]) / det
	S_inv[0][1] = (S[0][2]*S[2][1] - S[0][1]*S[2][2]) / det
	S_inv[0][2] = (S[0][1]*S[1][2] - S[0][2]*S[1][1]) / det
	S_inv[1][0] = (S[1][2]*S[2][0] - S[1][0]*S[2][2]) / det
	S_inv[1][1] = (S[0][0]*S[2][2] - S[0][2]*S[2][0]) / det
	S_inv[1][2] = (S[0][2]*S[1][0] - S[0][0]*S[1][2]) / det
	S_inv[2][0] = (S[1][0]*S[2][1] - S[1][1]*S[2][0]) / det
	S_inv[2][1] = (S[0][1]*S[2][0] - S[0][0]*S[2][1]) / det
	S_inv[2][2] = (S[0][0]*S[1][1] - S[0][1]*S[1][0]) / det
	
	// K = P*H'*S_inv (Kalman gain)
	var K [6][3]float64
	for i := 0; i < 6; i++ {
		for j := 0; j < meas_size; j++ {
			K[i][j] = 0
			for k := 0; k < 6; k++ {
				for l := 0; l < meas_size; l++ {
					K[i][j] += s.P[i][k] * H[l][k] * S_inv[l][j]
				}
			}
		}
	}
	
	// Update state: x = x + K*innovation
	for i := 0; i < 6; i++ {
		for j := 0; j < meas_size; j++ {
			s.x[i] += K[i][j] * innovation[j]
		}
	}
	
	// Update covariance: P = P - K*H*P
	var KH [6][6]float64
	for i := 0; i < 6; i++ {
		for j := 0; j < 6; j++ {
			KH[i][j] = 0
			for k := 0; k < meas_size; k++ {
				KH[i][j] += K[i][k] * H[k][j]
			}
		}
	}
	
	for i := 0; i < 6; i++ {
		for j := 0; j < 6; j++ {
			temp := 0.0
			for k := 0; k < 6; k++ {
				temp += KH[i][k] * s.P[k][j]
			}
			s.P[i][j] -= temp
		}
	}
	
	// Ensure P remains positive definite (add small diagonal term if needed)
	for i := 0; i < 6; i++ {
		if s.P[i][i] < 1e-12 {
			s.P[i][i] = 1e-12
		}
	}
	
	// Update attitude variables for compatibility
	s.roll = s.x[0]
	s.pitch = s.x[1]
	s.heading = s.x[2]
}

// updateDCM updates the DCM from current Euler angles
func (s *SimpleState) updateDCM() {
	s.dcm = eulerToDCM(s.x[0], s.x[1], s.x[2])
}


// updateDerivedQuantities calculates slip/skid, turn rate, g-load
func (s *SimpleState) updateDerivedQuantities(m *Measurement) {
	if !m.SValid {
		return
	}
	
	// G-load is the magnitude of acceleration
	ax, ay, az := m.A1, m.A2, m.A3
	s.gLoad = math.Sqrt(ax*ax + ay*ay + az*az)
	
	// Slip/skid angle (simplified)
	s.slipSkid = math.Atan2(-ay, az) * R2D
	
	// Turn rate from gyro (bias corrected)
	s.turnRate = ((m.B3*D2R) - s.x[5]) * R2D // Convert back to deg/s
}






































func (s *SimpleState) Predict(t float64) {
	return
}

// Update performs the AHRSSimple AHRS computations.
// The idea behind the Simple AHRS algorithm is to use the GPS to compute what the accelerometer should show
// and then to create a rotation matrix to map the measured accelerometer vector onto this vector and the
// speed vector (GPS Track) onto the sensor x-axis.  Then the gyro is used to further improve this estimate.
// This is really a poor-man's sensor fusion algorithm.  The proper way to do this is with a Kalman Filter,
// but this approach is simpler and easier to debug, and should be good enough for most flight conditions.
//
// It is a step on the way to the full Kalman Filter implementation, a bit more obvious what's going on so
// the math and stratux integration can be more easily developed and debugged.

// SimpleDCM implements a Direction Cosine Matrix AHRS algorithm

func (s *SimpleState) Update(m *Measurement) {


	dt := m.T - s.T      //sensor delta-T
	dtw := m.TW - s.tW   //gps delta-T

	if dt > maxDT || dtw > maxDT {
		s.init(m)
		return
	}
	
	// Extract sensor data (assuming Measurement has these fields)
	// You'll need to adjust these field names to match the actual Measurement struct
	gx := (m.B1 - s.gyroBias[0]) * D2R // Gyro X (rad/s) - bias corrected
	gy := (m.B2 - s.gyroBias[1]) * D2R // Gyro Y (rad/s) - bias corrected  
	gz := (m.B3 - s.gyroBias[2]) * D2R // Gyro Z (rad/s) - bias corrected
	ax := m.A1 // Accel X (g)
	ay := m.A2 // Accel Y (g)
	az := m.A3 // Accel Z (g)
	mx := m.M2 // Mag X 
	my := m.M1 // Mag Y 
	mz := -m.M3 // Mag Z 


	// Apply magnetometer calibration if enabled
    if s.magCalibrated {
        mx = (mx - s.magOffset[0]) * s.magScale[0]
        my = (my - s.magOffset[1]) * s.magScale[1]
        mz = (mz - s.magOffset[2]) * s.magScale[2]
    }
	
	mmag := math.Sqrt(mx*mx+my*my+mz*mz)

	//check if magnetometer data is invalid  (this works good!  Keeps outputs within calibration values)
	if mmag < 0.9 || mmag > 1.1 {
		mx, my, mz = 0, 0, 0
	}

	
	
	s.roll = math.Atan2(ay, az)
	s.pitch = math.Atan2(-ax, math.Sqrt(ay*ay + az*az))

	if mx!=0 && my!=0 && mz!=0 {
		//transform from body to inertial coordinates, pitch and roll only
		m1 := mx * math.Cos(s.pitch) + my * math.Sin(s.roll) * math.Sin(s.pitch) + mz * math.Cos(s.roll) * math.Sin(s.pitch);           
		m2 := my * math.Cos(s.roll) - mz * math.Sin(s.roll);
		s.heading = math.Atan2(m2, m1);
		for s.heading < 0 {
			s.heading += 2 * Pi
		}
		for s.heading >= 2*Pi {
			s.heading -= 2 * Pi
		}
	}

	//these were fixed to use body-fixed accelerations instead of inertial accelerations
	// Initialize Slip/Skid, Rate of Turn, and GLoad.
	s.slipSkid = math.Atan2(-m.A2, m.A3) * R2D
	s.turnRate = 0
	s.gLoad = m.A3 
	log.Printf("roll %f pitch %f yaw %f slip %f gload %f ax %f ay %f az %f gx %f gy %f gz %f mx %f my %f mz %f", s.roll*R2D, s.pitch*R2D, s.heading*R2D, s.slipSkid, s.gLoad, ax, ay, az, gx, gy, gz, mx, my, mz)
	
	updateLogMap(s, m, s.logMap)

	s.T = m.T
	s.tW = m.TW

}











// eulerToDCM calculates the Direction Cosine Matrix from Euler angles
// phi = roll, theta = pitch, psi = yaw (all in radians)
// Returns a 3x3 DCM that rotates from body frame to earth frame
func eulerToDCM(phi, theta, psi float64) [3][3]float64 {
    // Precompute trig functions
    cphi := math.Cos(phi)
    sphi := math.Sin(phi)
    ctheta := math.Cos(theta)
    stheta := math.Sin(theta)
    cpsi := math.Cos(psi)
    spsi := math.Sin(psi)
    
    // Standard aerospace DCM (Z-Y-X rotation sequence)
    // This is the most common convention for aircraft
    dcm := [3][3]float64{
        {ctheta * cpsi, ctheta * spsi, -stheta},
        {sphi * stheta * cpsi - cphi * spsi, sphi * stheta * spsi + cphi * cpsi, sphi * ctheta},
        {cphi * stheta * cpsi + sphi * spsi, cphi * stheta * spsi - sphi * cpsi, cphi * ctheta},
    }
    
    return dcm
}







































 





// normalizeMatrix orthonormalizes the DCM matrix
func normalizeMatrix(in [3][3]float64) (out [3][3]float64) {
	// Get first two rows as vectors
	t0 := [3]float64{in[0][0], in[0][1], in[0][2]}
	t1 := [3]float64{in[1][0], in[1][1], in[1][2]}
	
	// Normalize first row
	norm := math.Sqrt(t0[0]*t0[0] + t0[1]*t0[1] + t0[2]*t0[2])
	if norm > 0 {
		for i := 0; i < 3; i++ {
			t0[i] /= norm
		}
	}
	
	// Make second row orthogonal to first
	dot := t0[0]*t1[0] + t0[1]*t1[1] + t0[2]*t1[2]
	for i := 0; i < 3; i++ {
		t1[i] -= dot * t0[i]
	}
	
	// Normalize second row
	norm = math.Sqrt(t1[0]*t1[0] + t1[1]*t1[1] + t1[2]*t1[2])
	if norm > 0 {
		for i := 0; i < 3; i++ {
			t1[i] /= norm
		}
	}
	
	// Third row is cross product of first two
	t2 := [3]float64{
		t0[1]*t1[2] - t0[2]*t1[1],
		t0[2]*t1[0] - t0[0]*t1[2],
		t0[0]*t1[1] - t0[1]*t1[0],
	}
	
	// Copy back to DCM
	for i := 0; i < 3; i++ {
		out[0][i] = t0[i]
		out[1][i] = t1[i]
		out[2][i] = t2[i]
	}
	return out
}


func updateLogMap(s *SimpleState, m *Measurement, p map[string]interface{}) {
	var simpleLogMap = map[string]func(s *SimpleState, m *Measurement) float64{
		"Ta":                func(s *SimpleState, m *Measurement) float64 { return s.T },
		"TWa":               func(s *SimpleState, m *Measurement) float64 { return s.tW },
		"Roll":              func(s *SimpleState, m *Measurement) float64 { return s.roll * D2R },
		"Pitch":             func(s *SimpleState, m *Measurement) float64 { return s.pitch * D2R },
		"Heading":           func(s *SimpleState, m *Measurement) float64 { return s.heading * D2R },
		"T":          func(s *SimpleState, m *Measurement) float64 { return m.T },
		"TW":         func(s *SimpleState, m *Measurement) float64 { return m.TW },
		"A1":         func(s *SimpleState, m *Measurement) float64 { return m.A1 },
		"A2":         func(s *SimpleState, m *Measurement) float64 { return m.A2 },
		"A3":         func(s *SimpleState, m *Measurement) float64 { return m.A3 },
		"B1":         func(s *SimpleState, m *Measurement) float64 { return m.B1 },
		"B2":         func(s *SimpleState, m *Measurement) float64 { return m.B2 },
		"B3":         func(s *SimpleState, m *Measurement) float64 { return m.B3 },
		"M1":         func(s *SimpleState, m *Measurement) float64 { return m.M1 },
		"M2":         func(s *SimpleState, m *Measurement) float64 { return m.M2 },
		"M3":         func(s *SimpleState, m *Measurement) float64 { return m.M3 },
	}

	for k := range simpleLogMap {
		p[k] = simpleLogMap[k](s, m)
	}
}

var SimpleJSONConfig = `{
  "State": [
    ["Roll", "RollGPS", "RollGyr", "RollActual", 0],
    ["Pitch", "PitchGPS", "PitchGyr", "PitchActual", 0],
    ["Heading", "HeadingGPS", "HeadingGyr", "HeadingActual", null],
    ["turnRate", null, null, "turnRateActual", 0],
    ["gLoad", null, null, "gLoadActual", 1],
    ["slipSkid", null, null, "slipSkidActual", 0],
    ["GroundSpeed", null, null, null, 0],
    ["T", null, null, null, null],
    ["E0", "EGPS0", "EGyr0", "E0Actual", null],
    ["E1", "EGPS1", "EGyr1", "E1Actual", null],
    ["E2", "EGPS2", "EGyr2", "E2Actual", null],
    ["E3", "EGPS3", "EGyr3", "E3Actual", null],
    ["Z1", null, null, "Z1Actual", 0],
    ["Z2", null, null, "Z2Actual", 0],
    ["Z3", null, null, "Z3Actual", -1]
    ["C1", null, null, "C1Actual", 0],
    ["C2", null, null, "C2Actual", 0],
    ["C3", null, null, "C3Actual", 0]
    ["H1", null, null, "H1Actual", 0],
    ["H2", null, null, "H2Actual", 0],
    ["H3", null, null, "H3Actual", 0]
    ["D1", null, null, "D1Actual", 0],
    ["D2", null, null, "D2Actual", 0],
    ["D3", null, null, "D3Actual", 0]
  ],
  "Measurement": [
    ["W1", "W1a", 0],
    ["W2", "W2a", 0],
    ["W3", "W3a", 0],
    ["A1", null, 0],
    ["A2", null, 0],
    ["A3", null, 0],
    ["B1", null, 0],
    ["B2", null, 0],
    ["B3", null, 0]
    ["M1", null, 0],
    ["M2", null, 0],
    ["M3", null, 0]
  ]
}`





















































































// Valid returns whether the current state is a valid estimate or if something went wrong in the calculation.
func (s *SimpleState) Valid() (ok bool) {
	return true
}


// Reset restarts the algorithm from scratch.
func (s *SimpleState) Reset() {
	s.needsInitialization = true
}

// RollPitchHeading returns the current attitude values as estimated by the Kalman algorithm.
func (s *SimpleState) RollPitchHeading() (roll float64, pitch float64, heading float64) {
	roll, pitch, heading = s.roll, -s.pitch, s.heading
	return
}


// MagHeading returns the magnetic heading in degrees.
func (s *SimpleState) MagHeading() (hdg float64) {

	return s.heading
}

// SlipSkid returns the slip/skid angle in degrees.
func (s *SimpleState) SlipSkid() (slipSkid float64) {
	return s.slipSkid
}

// RateOfTurn returns the turn rate in degrees per second.
func (s *SimpleState) RateOfTurn() (turnRate float64) {
	return 0
}

// GLoad returns the current G load, in G's.
func (s *SimpleState) GLoad() (gLoad float64) {
	return s.gLoad
}

// GetState returns the state of the system
func (s *SimpleState) GetState() *State {
	return &s.State
}

// GetLogMap returns a map providing current state and measurement values for analysis
func (s *SimpleState) GetLogMap() (p map[string]interface{}) {
	return s.logMap
}


// AHRSProvider defines an AHRS (Kalman or other) algorithm, such as ahrs_kalman, ahrs_simple, etc.
type AHRSProvider interface {
	// Compute runs both the "predict" and "update" stages of the algorithm, for convenience.
	Compute(m *Measurement)
	// SetSensorQuaternion changes the AHRS algorithm's sensor quaternion F.
	Valid() bool
	// Reset restarts the algorithm from scratch.
	Reset()
	// RollPitchHeading returns the current attitude values as estimated by the Kalman algorithm.
	RollPitchHeading() (roll float64, pitch float64, heading float64)
	// MagHeading returns the current magnetic heading in degrees as estimated by the Kalman algorithm.
	MagHeading() (hdg float64)
	// SlipSkid returns the slip/skid angle in degrees as estimated by the Kalman algorithm.
	SlipSkid() (slipSkid float64)
	// RateOfTurn returns the turn rate in degrees per second as estimated by the Kalman algorithm.
	RateOfTurn() (turnRate float64)
	// GLoad returns the current G load, in G's as estimated by the Kalman algorithm.
	GLoad() (gLoad float64)
	// GetState returns all the information about the current state.
	GetState() 
	// GetLogMap returns a map customized for each AHRSProvider algorithm to provide more detailed information
	// for debugging and logging.
	GetLogMap() map[string]interface{}
}


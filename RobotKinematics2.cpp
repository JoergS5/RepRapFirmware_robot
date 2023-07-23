/*
 * RobotKinematics2.cpp
 *
 *  Created on: 23.07.2023
 *      Author: JoergS5
 */

#include <Movement/Kinematics/RobotKinematics.h>

#if SUPPORT_ROBOT

/*
 * angles is in degrees (rotary axis) or mm (linear axis)
 * angles are sorted by chain, i.e. CAZXY
 */

void RobotKinematics::getForwardBySkew(const float *angles, float *mxTo) const noexcept {
	for(int i=0; i < numOfAxes; i++) {
		if(forwardChainCompressed[i] == '.') {
			getForwardSpecialParts(angles, i, mxTo);
		}
		else {
			if(axisTypes[i] == 'R') {
				int rotarynr = getRotaryIndex(i);
				getRodrigues2_Rot(&cache[i*3 + offsetScrewOmega], &cache[i*3+ offsetScrewQ],
						&cache[rotarynr*12 + offsetScrewOmega2], &cache[rotarynr*3 + offsetScrewV],
						(angles[i] - cache[offsetMreference+i]) / radiansToDegrees, mxTemp);
			}
			else {
				getRodrigues2_Pris(&cache[i*3+ offsetScrewOmega], angles[i] - cache[offsetMreference+i], mxTemp);
			}
			if(i == 0) {
				memcpy(mxTo, mxTemp, sizeof(float) * 12);
			}
			else {
				multiplyRotationMatrixInplaceLeft(mxTo, mxTemp);
			}
		}
	}
	multiplyRotationMatrixInplaceLeft(mxTo, screw_M);
}

/*
 * process forwardChainSpecial letters with special subkinematic
 */

void RobotKinematics::getForwardSpecialParts(const float *angles, int i, float *mxTo) const noexcept {
	if(specialMethod == 1) { // CoreXY
		char letter = getLetterInChain(i);
		if(letter == 'X') { // first letter => process both in one step
			float angleX = (angles[i] + angles[i+1]); // A+B
			float angleY = (angles[i] - angles[i+1]); // A-B

			getRodrigues2_Pris(&cache[i*3+ offsetScrewOmega], angleX, mxTemp);
			multiplyRotationMatrixInplaceLeft(mxTo, mxTemp);

			getRodrigues2_Pris(&cache[(i+1)*3+ offsetScrewOmega], angleY, mxTemp);
			multiplyRotationMatrixInplaceLeft(mxTo, mxTemp);

		}
	}

}

void RobotKinematics::getRodrigues2_Pris(float *screwOmega, float dist, float *_mxTemp) const noexcept {
	_mxTemp[0] = 1.0;
	_mxTemp[4] = 0.0;
	_mxTemp[8] = 0.0;
	_mxTemp[1] = 0.0;
	_mxTemp[5] = 1.0;
	_mxTemp[9] = 0.0;
	_mxTemp[2] = 0.0;
	_mxTemp[6] = 0.0;
	_mxTemp[10] = 1.0;

	_mxTemp[3] = screwOmega[0] * dist;
	_mxTemp[7] = screwOmega[1] * dist;
	_mxTemp[11] = screwOmega[2] * dist;

}

/*
 * create transformation matrix based on exponential screw/skew values
 * version without using cache
 *
 * \param screw 6 parameters: 3 skew, 3 q positions on axis
 *
 * \param theta in radians
 *
 * \param T resulting transformation matrix 3x4
 */

void RobotKinematics::getRodrigues(float *screwO, float *screwQ, float theta,
		float *mx, bool isRotational) const noexcept {
	if(isRotational) {
		float v[3], omega2[9];
		setRodrigues1(screwO, screwQ, omega2, v);
		getRodrigues2_Rot(screwO, screwQ, omega2, v, theta, mx);
	}
	else {
		getRodrigues2_Pris(screwO, theta, mx);
	}
}

/*
 * prepare Rodrigues part 1, the parts which do not depend on theta
 *
 * the result is cached:
 * v is the cross product of w and q
 * omega2 is omega squared, a 3x3 matrix
 */

void RobotKinematics::setRodrigues1(float *screwO, float *screwQ, float *omega2, float *v) const noexcept {
	// v = - w x q
	v[0] = -screwO[1]*screwQ[2] + screwO[2]*screwQ[1];
	v[1] = -screwO[2]*screwQ[0] + screwO[0]*screwQ[2];
	v[2] = -screwO[0]*screwQ[1] + screwO[1]*screwQ[0];

	float omega[9];
	omega[0] = 0.0;
	omega[4] = 0.0;
	omega[8] = 0.0;

	omega[1] = - screwO[2];
	omega[2] = screwO[1];
	omega[3] = screwO[2];
	omega[5] = - screwO[0];
	omega[6] = - screwO[1];
	omega[7] = screwO[0];

	multiplyMatrix(3, 3, omega, 3, omega, omega2);
}

/*
 * Rodrigues part 2, calculating T from known properties and theta
 */

void RobotKinematics::getRodrigues2_Rot(const float *screwO, const float *screwQ_notused, const float *omega2,
		const float *v, float theta, float *mx) const noexcept {
	float sinTheta = 0.0, oneMinCos = 0.0, thetaMinSinTheta = 0.0;
	if(theta != 0.0) {
		sinTheta = sinf(theta);
		oneMinCos = 1.0f - cosf(theta);
		thetaMinSinTheta = theta - sinTheta;
	}

	// calculate rotations:
	mx[0] = 1.0f + oneMinCos * omega2[0];
	mx[1] = sinTheta * (- screwO[2]) + oneMinCos * omega2[1];
	mx[2] = sinTheta * screwO[1] + oneMinCos * omega2[2];

	mx[4] = sinTheta * screwO[2] + oneMinCos * omega2[3];
	mx[5] = 1.0f + oneMinCos * omega2[4];
	mx[6] = sinTheta * (- screwO[0]) + oneMinCos * omega2[5];

	mx[8] = sinTheta * (- screwO[1]) + oneMinCos * omega2[6];
	mx[9] = sinTheta * screwO[0] + oneMinCos * omega2[7];
	mx[10] = 1.0f + oneMinCos * omega2[8];

	// calculate positions: Lynch/Park page 105, alternative method see Murray/Li/Sastry page 42
	// Pardos-Gotor page 42
	mx[3] = (thetaMinSinTheta * omega2[0] + theta) * v[0] +
			(oneMinCos * (- screwO[2]) + thetaMinSinTheta * omega2[1]) *  v[1] +
			(oneMinCos * screwO[1] + thetaMinSinTheta * omega2[2])  * v[2];
	mx[7] = (oneMinCos * screwO[2] + thetaMinSinTheta * omega2[3]) * v[0] +
			(thetaMinSinTheta * omega2[4] + theta) *  v[1] +
			(oneMinCos * (- screwO[0]) + thetaMinSinTheta * omega2[5])  * v[2];
	mx[11] = (oneMinCos * (- screwO[1]) + thetaMinSinTheta * omega2[6]) * v[0] +
			(oneMinCos * screwO[0] + thetaMinSinTheta * omega2[7]) *  v[1] +
			(thetaMinSinTheta * omega2[8] + theta)  * v[2];
}

/* store Mnoap and its inverse
 *
 * Mnaop is the endpoint for given actuator angles/positions and default tool length/orientation
 *
 * \param values: x-axis, y-axis, z-axis, pos values as 3-value vectors each, in this order
 *
 */

void RobotKinematics::setSkew_M(float *values) const noexcept {
	screw_M[0] = values[0];
	screw_M[4] = values[1];
	screw_M[8] = values[2];

	screw_M[1] = values[3];
	screw_M[5] = values[4];
	screw_M[9] = values[5];

	screw_M[2] = values[6];
	screw_M[6] = values[7];
	screw_M[10] = values[8];

	screw_M[3] = values[9];
	screw_M[7] = values[10];
	screw_M[11] = values[11];

	normalizeVector(screw_M[0], screw_M[4], screw_M[8]);
	normalizeVector(screw_M[1], screw_M[5], screw_M[9]);
	normalizeVector(screw_M[2], screw_M[6], screw_M[10]);

	recalcMInv();
}

void RobotKinematics::recalcMInv() const noexcept {
	//MT
	screw_MInv[0] = screw_M[0];	screw_MInv[4] = screw_M[1];	screw_MInv[8] = screw_M[2];
	screw_MInv[1] = screw_M[4];	screw_MInv[5] = screw_M[5];	screw_MInv[9] = screw_M[6];
	screw_MInv[2] = screw_M[8];	screw_MInv[6] = screw_M[9];	screw_MInv[10] = screw_M[10];
	//-MT*t
	screw_MInv[3] = -(screw_MInv[0]*screw_M[3] + screw_MInv[1]*screw_M[7] + screw_MInv[2]*screw_M[11]);
	screw_MInv[7] = -(screw_MInv[4]*screw_M[3] + screw_MInv[5]*screw_M[7] + screw_MInv[6]*screw_M[11]);
	screw_MInv[11] = -(screw_MInv[8]*screw_M[3] + screw_MInv[9]*screw_M[7] + screw_MInv[10]*screw_M[11]);
}

int RobotKinematics::getRotaryIndex(int axisnr) const noexcept {
	int ct = 0;
	for(int i=0; i < numOfAxes; i++) {
		if(axisTypes[i] == 'R') {
			if(i == axisnr) {
				return ct;
			}
			ct++;
		}
	}
	return -1; // not found or axisnr wrong
}

/*
 * multiply two matrices
 *
 * \param m, r, n rows first, cols first (== rows second), cols second
 * \param mx1 first matrix in format m x r
 * \param mx2 second matrix in format r x n
 * \param mxTo result in format m x n
 *
 * cost: m*n*r(A,M) (m*n*r additions and m*n*r multiplications), T would be for trigonometric
 */

void RobotKinematics::multiplyMatrix(int m, int r, const float *mx1, int n, const float *mx2, float *mxTo) const noexcept {
	for(int row = 0; row < m; row++) {
		for(int col = 0; col < n; col++) {
			mxTo[row * n + col] = 0.0f;
			for(int e = 0; e < r; e++) {
				mxTo[row * n + col] += mx1[row * r + e] * mx2[e * n + col];
			}
		}
	}
}

/*
 * multiply transformation matrix 3x4 inplace by using a temp array storing the original rows of m1
 * faster than multiplyRotationMatrix, because no memcpy and temp uses 4 instead of 12 floats
 * result is in m1
 *
 * \param m1 first 3x4 matrix which will contain the multiplication result
 *
 * \param m2 second 3x4 matrix
 *
 * \param temp4 temporary 4 element array to hold original row values of m1 which are overwritten
 *
 * unit tested
 */

void RobotKinematics::multiplyRotationMatrixInplaceLeft(float *m1, const float *m2) const noexcept {
	for(int row = 0; row < 3; row++) {
		for(int col=0; col < 4; col++) { // store current row of m1
			temp4[col] = m1[row * 4 + col];
		} // don't merge the two for loops
		for(int col = 0; col < 4; col++) {
			m1[row * 4 + col] = temp4[0]*m2[col] + temp4[1]*m2[4+col] + temp4[2]*m2[8+col];
			if(col == 3) { // add fourth line last column of (0 0 0 1)
				m1[row * 4 + col] += temp4[3];
			}
		}
	}
}

void RobotKinematics::rotationMatrixInverse(const float *mx, float *mxInverse) const noexcept {
	//MT
	mxInverse[0] = mx[0];	mxInverse[4] = mx[1];	mxInverse[8] = mx[2];
	mxInverse[1] = mx[4];	mxInverse[5] = mx[5];	mxInverse[9] = mx[6];
	mxInverse[2] = mx[8];	mxInverse[6] = mx[9];	mxInverse[10] = mx[10];
	//-MT*t
	mxInverse[3] =  -(mxInverse[0]*mx[3] + mxInverse[1]*mx[7] + mxInverse[2]*mx[11]);
	mxInverse[7] =  -(mxInverse[4]*mx[3] + mxInverse[5]*mx[7] + mxInverse[6]*mx[11]);
	mxInverse[11] = -(mxInverse[8]*mx[3] + mxInverse[9]*mx[7] + mxInverse[10]*mx[11]);
}

/*
 * multiply two rotation matrices. memory efficient by sparing fourth line and assume last line to be (0 0 0 1)
 * matrices have 3 rows, 4 cols each
 *
 * m1, m2 and result must exist and be 3x4 each
 *
 * \param m1, m2 matrices
 *
 * \param result resulting 3x4 matrix (in reality 4x4 matrix)
 */

void RobotKinematics::multiplyRotationMatrix(const float *m1, const float *m2, float *result) const noexcept {
	for(int row = 0; row < 3; row++) { // last row not calculated, is always 0 0 0 1
		for(int col = 0; col < 4; col++) {
			result[row*4 + col] = 0.0f;
			for(int e=0; e < 3; e++) { // last element handled special (0 0 0 1)
				result[row*4 + col] += m1[row*4 + e] * m2[e*4 + col];
			}
			if(col == 3) { // only for position col last element is * 1.0
				result[row*4 + col] += m1[row*4 + 3];
			}
		}
	}
}

/*
 * get inverse
 */
void RobotKinematics::getInverseBySkew(const float *mxTo, float *anglesTo) const noexcept {
	//tempS20.copy(axisTypes);
	if(specialMethod == 1) { // CoreXY AC or BC
		int posA = getPositionOfLetterInChain('A');
		if(posA >= 0) {
			getInverseAC(mxTo, anglesTo, true);
		}
		else {
			getInverseAC(mxTo, anglesTo, false);
		}
		getInverseCoreXY_XYZ(mxTo, anglesTo, true);
	}
	else {
		int posA = getPositionOfLetterInChain('A');
		if(posA >= 0) {
			getInverseAC(mxTo, anglesTo, true);
		}
		else {
			getInverseAC(mxTo, anglesTo, false);
		}
		getInverseCoreXY_XYZ(mxTo, anglesTo, false);
	}
}

/*
 * recover CA or CB
 * acMode true = AC, false = BC
 *
 * anglesCA: 0 has C, 1 has A/B. Array can be larger, but size 2 at least
 */

void RobotKinematics::getInverseAC(const float *mxTo, float *anglesCA, bool acMode) const noexcept {
	if(mxTo[2] == 0.0 && mxTo[6] == 0.0 && mxTo[3] == 0.0 && mxTo[7] == 0.0) {
		anglesCA[0] = 0.0;
	}
	else {
		if(acMode) {
			anglesCA[0] = atan2f(mxTo[2], -mxTo[6]) * radiansToDegrees; // C
		}
		else {
			anglesCA[0] = atan2f(mxTo[6], mxTo[2]) * radiansToDegrees; // C
		}
	}

	anglesCA[1] = acosf(mxTo[10]) * radiansToDegrees; // A/B

	if(anglesCA[0] == 0.0) {
		// take this result
	}
	else if(anglesCA[0] > 0 && abSign == 0) {
		// take this result
	}
	else if(abSign == 1 && anglesCA[0] < 0) {
		// also ok
	}
	else {
		if(anglesCA[0] < 0) {
			anglesCA[0] += 180.0;
			anglesCA[1] = - anglesCA[1];
		}
		else {
			anglesCA[0] -= 180.0;
			anglesCA[1] = - anglesCA[1];
		}
	}

}

void RobotKinematics::getInverseCoreXY_XYZ(const float *mxTo, float *anglesResult, bool iscorexy)
		const noexcept {

	// anglesResult 0 is C, 1 is A, already calculated

	// solution without correction of position effect of rotations:
	float anglesNew[5]; // order CAZXY
	anglesNew[0] = anglesResult[0];
	anglesNew[1] = anglesResult[1];
	anglesNew[2] = mxTo[11];
	if(iscorexy) {
		anglesNew[3] = 0.5*(mxTo[3] + mxTo[7]);
		anglesNew[4] = 0.5*(mxTo[3] - mxTo[7]);
	}
	else {
		anglesNew[3] = mxTo[3];
		anglesNew[4] = mxTo[7];
	}
	float mxXYZ[12];
	getForwardBySkew(anglesNew, mxXYZ);

	// effect of the rotations:
	float diffX = mxXYZ[3] - mxTo[3];
	float diffY = mxXYZ[7] - mxTo[7];
	float diffZ = mxXYZ[11] - mxTo[11];

	// correct angles XYZ:
	float xyz[3];
	xyz[0] = mxTo[11] + diffZ;
	xyz[1] = mxTo[3] + diffX;
	xyz[2] = mxTo[7] + diffY;

	// subtract reference values and change values for CoreXY:
	anglesResult[2] = xyz[0] - cache[offsetMreference+2]; // Z
	if(iscorexy) {
		float x = xyz[1] - cache[offsetMreference+3];
		float y = xyz[2] - cache[offsetMreference+4];
		anglesResult[3] = 0.5*(x + y);
		anglesResult[4] = 0.5*(x - y);
	}
	else {
		anglesResult[3] = xyz[1] - cache[offsetMreference+3]; // X
		anglesResult[4] = xyz[2] - cache[offsetMreference+4]; // Y
	}
	anglesResult[0] -= cache[offsetMreference]; //C
	anglesResult[1] -= cache[offsetMreference+1]; //A

}

void RobotKinematics::normalizeVector(float &x, float &y, float &z) const noexcept {
	if(x == 0.0 && y == 0.0 && z == 1.0) return;
	if(x == 0.0 && y == 1.0 && z == 0.0) return;
	if(x == 1.0 && y == 0.0 && z == 0.0) return;
	float len = sqrtf(x*x + y*y + z*z);
	if(len != 0.0) {
		x /= len;
		y /= len;
		z /= len;
	}
}

/*
 * for inverse: translate G-Code into mx matrix (only z-axis and pos relevant)
 */
void RobotKinematics::XYZACTomx(const float *xyzac, float *mx) const noexcept {
	mx[3] = xyzac[0];
	mx[7] = xyzac[1];
	mx[11] = xyzac[2];

	mx[2] = sin(xyzac[3]/radiansToDegrees) * sin(xyzac[4]/radiansToDegrees);
	mx[6] = - cos(xyzac[4]/radiansToDegrees) * sin(xyzac[3]/radiansToDegrees);
	mx[10] = cos(xyzac[3]/radiansToDegrees);	// cA
}

/*
 * for inverse: translate G-Code into mx matrix (only z-axis and pos relevant)
 */
void RobotKinematics::XYZBCTomx(const float *xyzbc, float *mx) const noexcept {
	mx[3] = xyzbc[0];
	mx[7] = xyzbc[1];
	mx[11] = xyzbc[2];

	mx[2] = cos(xyzbc[4]/radiansToDegrees) * sin(xyzbc[3]/radiansToDegrees);
	mx[6] = sin(xyzbc[3]/radiansToDegrees) * sin(xyzbc[4]/radiansToDegrees);
	mx[10] = cos(xyzbc[3]/radiansToDegrees);	// cosB
}

/*
 * for forward: translate mx matrix into XYZAC pos/angles (only z axis and pos of mx relevant)
 */
void RobotKinematics::mxToXYZAC(const float *mx, float *xyzac) const noexcept {
	xyzac[0] = mx[3];
	xyzac[1] = mx[7];
	xyzac[2] = mx[11];

	bool acMode = true;
	float anglesCA[2];
	getInverseAC(mx, anglesCA, acMode);
	xyzac[3] = anglesCA[1]; // A
	xyzac[4] = anglesCA[0]; // C
}

/*
 * for forward: translate mx matrix into XYZBC pos/angles (only z axis and pos of mx relevant)
 */
void RobotKinematics::mxToXYZBC(const float *mx, float *xyzbc) const noexcept {
	xyzbc[0] = mx[3];
	xyzbc[1] = mx[7];
	xyzbc[2] = mx[11];

	bool acMode = false;
	float anglesCB[5];
	getInverseAC(mx, anglesCB, acMode);
	xyzbc[3] = anglesCB[1]; // B
	xyzbc[4] = anglesCB[0]; // C
}


#endif



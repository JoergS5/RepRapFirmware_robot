/*
 * RobotKinematics.cpp
 *
 *  Created on: 10.06.2023
 *      Author: JoergS5
 */


#include <Movement/Kinematics/RobotKinematics.h>
#include <GCodes/GCodeBuffer/GCodeBuffer.h>
#include <Platform/RepRap.h>
#include <Movement/DDA.h>

#if SUPPORT_OBJECT_MODEL

// Object model table and functions
// Note: if using GCC version 7.3.1 20180622 and lambda functions are used in this table, you must compile this file with option -std=gnu++17.
// Otherwise the table will be allocated in RAM instead of flash, which wastes too much RAM.

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(...) OBJECT_MODEL_FUNC_BODY(RobotKinematics, __VA_ARGS__)

constexpr ObjectModelTableEntry RobotKinematics::objectModelTable[] =
{
	// Within each group, these entries must be in alphabetical order
	// 0. kinematics members
	{ "name",	OBJECT_MODEL_FUNC(self->GetName(true)), 	ObjectModelEntryFlags::none },
	//TODO lots more to be added here
};

constexpr uint8_t RobotKinematics::objectModelTableDescriptor[] = { 1, 1 };

DEFINE_GET_OBJECT_MODEL_TABLE_WITH_PARENT(RobotKinematics, ZLeadscrewKinematics)

#endif


RobotKinematics::RobotKinematics() noexcept
	: ZLeadscrewKinematics(KinematicsType::scara, SegmentationType(true, false, false))
{
}

RobotKinematics::~RobotKinematics() {
}

const char *RobotKinematics::GetName(bool forStatusReport) const noexcept {
	return "robot";
}

bool RobotKinematics::Configure(unsigned int mCode, GCodeBuffer& gb, const StringRef& reply,
		bool& error) THROWS(GCodeException) {

	if (mCode == 669)
	{
		bool seen = false;
		String<50> inputValue;
		bool allowEmpty = false;

		if (gb.Seen('B')) {
			gb.TryGetQuotedString('B', inputValue.GetRef(), seen, allowEmpty);
			setB(inputValue.c_str());
			seen = true;
		}

		if (gb.Seen('A')) {
			gb.TryGetQuotedString('A', inputValue.GetRef(), seen, allowEmpty);
			setA(inputValue.c_str());
			seen = true;
		}

		if (gb.Seen('P')) {
			gb.TryGetQuotedString('P', inputValue.GetRef(), seen, allowEmpty);
			setP(inputValue.c_str());
			seen = true;
		}

		if(!seen) {
		}

		return seen;
	}
	else
	{
		return ZLeadscrewKinematics::Configure(mCode, gb, reply, error);
	}
}

bool RobotKinematics::CartesianToMotorSteps(const float machinePos[], const float stepsPerMm[],
		size_t numVisibleAxes, size_t numTotalAxes, int32_t motorPos[], bool isCoordinated) const noexcept {

	float mx[12];
	XYZACTomx(machinePos, mx);

	float anglesTo[numOfAxes];
	getInverseBySkew(mx, anglesTo);

	for(int i=0; i < numOfAxes; i++) {
		motorPos[i] = lrintf(anglesTo[i] * stepsPerMm[i]);
	}
	for (size_t axis = XYZ_AXES; axis < numVisibleAxes; ++axis)	{
		motorPos[axis] = lrintf(machinePos[axis] * stepsPerMm[axis]);
	}

	return true;
}

void RobotKinematics::MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[],
		size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const noexcept {

	float angles[numOfAxes];
	for(int i=0; i < numOfAxes; i++) {
		angles[i] = ((float)motorPos[i]/stepsPerMm[i]);
	}

	float mx[12];
	getForwardBySkew(angles, mx);

	float xyzac[5];
	mxToXYZAC(mx, xyzac);

	for(int i=0; i < 5; i++) {
		machinePos[i] = xyzac[i];
	}
}

bool RobotKinematics::IsReachable(float axesCoords[MaxAxes], AxesBitmap axes) const noexcept {
	return true;
}

LimitPositionResult RobotKinematics::LimitPosition(float coords[], const float * null initialCoords,
		size_t numVisibleAxes, AxesBitmap axesToLimit, bool isCoordinated, bool applyM208Limits) const noexcept {

	const bool m208Limited = applyM208Limits && Kinematics::LimitPositionFromAxis(coords, 0, numVisibleAxes, axesToLimit);
	return (m208Limited) ? LimitPositionResult::adjusted : LimitPositionResult::ok;

}

void RobotKinematics::GetAssumedInitialPosition(size_t numAxes, float positions[]) const noexcept {

}

AxesBitmap RobotKinematics::AxesAssumedHomed(AxesBitmap g92Axes) const noexcept {
	return g92Axes;
}

AxesBitmap RobotKinematics::MustBeHomedAxes(AxesBitmap axesMoving, bool disallowMovesBeforeHoming) const noexcept {
	return axesMoving;
}


AxesBitmap RobotKinematics::GetHomingFileName(AxesBitmap toBeHomed, AxesBitmap alreadyHomed,
		size_t numVisibleAxes, const StringRef& filename) const noexcept {
	AxesBitmap ret = Kinematics::GetHomingFileName(toBeHomed, alreadyHomed, numVisibleAxes, filename);
	filename.copy(HomeRobotFileName);
	return ret;

}

bool RobotKinematics::QueryTerminateHomingMove(size_t axis) const noexcept {
	return false;
}

void RobotKinematics::OnHomingSwitchTriggered(size_t axis, bool highEnd,
		const float stepsPerMm[], DDA& dda) const noexcept {

	float home = cache[offsetAngleLimits + axis*3 + 2];
	dda.SetDriveCoordinate(lrintf(home * stepsPerMm[axis]), axis);

//	const float hitPoint = ((highEnd) ? reprap.GetPlatform().AxisMaximum(axis) :
//			reprap.GetPlatform().AxisMinimum(axis));
//	dda.SetDriveCoordinate(lrintf(hitPoint * stepsPerMm[axis]), axis);

}

bool RobotKinematics::IsContinuousRotationAxis(size_t axis) const noexcept {
	return false;
}

AxesBitmap RobotKinematics::GetLinearAxes() const noexcept {
	return AxesBitmap::MakeFromBits(Z_AXIS);
}

AxesBitmap RobotKinematics::GetConnectedAxes(size_t axis) const noexcept {
	return (axis == X_AXIS || axis == Y_AXIS)
			? XyAxes
				: AxesBitmap::MakeFromBits(axis);

}

void RobotKinematics::LimitSpeedAndAcceleration(DDA& dda, const float *normalisedDirectionVector,
		size_t numVisibleAxes, bool continuousRotationShortcut) const noexcept {

}

/*
 * RobotKinematics1.cpp config
 *
 *  Created on: 10.06.2023
 *      Author: JoergS5
 */

#include <Movement/Kinematics/RobotKinematics.h>
#include <Platform/RepRap.h>
#include <Platform/Platform.h>
#include <Platform/Tasks.h>

void RobotKinematics::setB(const char* _robottype) const noexcept {
	tempS20.copy(_robottype);
	if(tempS20.Contains("CoreXY5AC") == 0 || tempS20.Contains("CAZ_corexy(YX)") == 0
			|| tempS20.Contains("CAZ_corexy(XY)") == 0) {
		setAxisTypes("RRPPP"); // sets numOfAxes
		setForwardChain("CAZ_corexy(XY)");
	}
	else if(tempS20.Contains("CoreXY5BC") == 0 || tempS20.Contains("CBZ_corexy(YX)") == 0
			|| tempS20.Contains("CBZ_corexy(XY)") == 0) {
		setAxisTypes("RRPPP"); // sets numOfAxes
		setForwardChain("CBZ_corexy(XY)");
	}
	else if(tempS20.Contains("Industrial6") == 0) {
		setAxisTypes("RRRRRR"); // sets numOfAxes
		setForwardChain("RRRRRR");
	}
	else if(tempS20.Contains("Prusa5BC") == 0) {
		setAxisTypes("RRPPP");
		setForwardChain("CBYZX");
	}
	else {
		setForwardChain(tempS20.c_str());
		setAxisTypes("RRPPP"); // AC with 3 linear axes as default
	}
	initCache();
}

void RobotKinematics::initCache() const noexcept {
	int numOfRAxes = 0;
	for(int i=0; i < numOfAxes; i++) {
		if(axisTypes[i] == 'R') {
			numOfRAxes++;
		}
	}

	for(size_t i=0; i < CACHESIZE; i++) {
		cache[i] = 0.0;
	}
	offsetScrewOmega = 0;
	offsetScrewQ = numOfAxes * 3;
	offsetScrewOmega2 = offsetScrewQ + numOfAxes * 3;
	offsetScrewV = offsetScrewOmega2 + numOfRAxes*12;
	offsetMreference = offsetScrewV + numOfRAxes*3;
	offsetAngleLimits = offsetMreference + numOfAxes;
	offsetStartOfInverse = offsetAngleLimits + numOfAxes*3;
}

/*
 * calculated values of forwardChainCompressed and forwardChainSpecial, set specialMethod
 */
// tested
void RobotKinematics::setForwardChain(const char* value) const noexcept {
	tempS20.copy(value);
	for(int i=0; i < 20; i++) {
		forwardChain[i] = value[i];
		if(value[i] == '\0') {
			break;
		}
	}
	checkAndSetSpecial("corexy(", 1);
	checkAndSetSpecial("corexz(", 2);
	checkAndSetSpecial("lindelta(", 15);
	checkAndSetSpecial("rotdelta(", 10);
	checkAndSetSpecial("5bar(", 9);
	checkAndSetSpecial("pall(", 14);
}

/*
 * set forwardChainCompressed: uppercase letters for normal operation, lowercase for special kinematics
 * specialMethod is also set, values are from K numbers if available
 */
void RobotKinematics::checkAndSetSpecial(const char *checkfor, int specialnr) const noexcept {
	int pos = tempS20.Contains(checkfor);

	int len = 0;
	for(int i=0; i < 20; i++) {
		if(checkfor[i] == '\0') {
			break;
		}
		len++;
	}

	int idx = 0;
	if(pos >= 0) {
		addNormalLetters(0,pos,idx);
		int offsetAfter = extractSpecial(pos+len, idx);
		addNormalLetters(offsetAfter,20,idx);
		forwardChainCompressed[idx] = '\0';
		specialMethod = 1;
	}

}

void RobotKinematics::addNormalLetters(int start, int stop,int &idx) const noexcept {
	for(int i=start; i < stop; i++) {
		if(forwardChain[i] == 'X' || forwardChain[i] == 'Y' || forwardChain[i] == 'Z'
				|| forwardChain[i] == 'A' || forwardChain[i] == 'B' || forwardChain[i] == 'C') {
			forwardChainCompressed[idx] = forwardChain[i];
			idx++;
		}
		else if(forwardChain[i] == '_') {
			// ignore
		}
		else if(forwardChain[i] == '\0') {
			break;
		}
	}
}

// return index after extraction
int RobotKinematics::extractSpecial(int start, int &idx) const noexcept {
	int ct = 0;
	int idxSpecial = 0;
	for(int i=start; i < 20; i++) {
		if(forwardChain[i] == ')' || forwardChain[i] == '\0') {
			if(forwardChain[i] == ')') {
				ct++;
			}
			break;
		}
		forwardChainCompressed[idx] = '.'; // placeholder for special
		forwardChainSpecial[idxSpecial] = forwardChain[i];
		idxSpecial++;
		ct++;
		idx++;
	}
	forwardChainSpecial[idxSpecial] = '\0';
	return start + ct;
}

// order is according to chain order (Open5x: BCYZX / UVYZX)
void RobotKinematics::setAxisTypes(const char* types) const noexcept {
	tempS20.copy(types);
	int len = int(tempS20.strlen());
	for(int i=0; i < len; i++) {
		axisTypes[i] = types[i];
	}
	axisTypes[len] = '\0';
	setNumOfAxes();
}

void RobotKinematics::errorMessage(const StringRef &msg) const noexcept {
	reprap.GetPlatform().Message(ErrorMessage, msg.c_str());
}


float RobotKinematics::getFloatOfElement(const char *value, char separator, int startpos, int idx) const noexcept {
	char v[20];
	int curSep = 0;
	int curPos = 0;
	for(int i=startpos; i < 200; i++) {
		if(value[i] == '\0' || curSep > idx) {
			v[curPos] = '\0';
			break;
		}
		else if(value[i] == separator) {
			curSep++;
		}
		else if(curSep == idx) {
			v[curPos] = value[i];
			curPos++;
		}
	}
	return SafeStrtof(v);
}

/*
 * set numOfAxes, calculated from axisTypes
 */
void RobotKinematics::setNumOfAxes() const noexcept {
	int ct = 0;
	for(size_t i=0; i < MAXNUMOFAXES+1; i++) {
		if(axisTypes[i] == '\0') {
			break;
		}
		ct++;
	}
	numOfAxes = ct;
}

void RobotKinematics::setC(const char *value) const noexcept {
	tempS20.copy(value); // will cut value if too long
	int startpos = -1; // position after the beginning marker "...="
	if(tempS20.Contains("Mnoap=") == 0) {
		int ct = getValueCount(value, "Mnoap=", ':', startpos);
		if(ct == 12) {
			float values[12];
			for(int i=0; i < 12; i++) {
				values[i] = getFloatOfElement(value, ':', startpos, i);
			}
			setSkew_M(values);
		}
	}
	else if(tempS20.Contains("Mreference=") == 0) {
		int ct = getValueCount(value, "Mreference=", ':', startpos);
		if(ct == numOfAxes) {
			for(int i=0; i < numOfAxes; i++) {
				cache[offsetMreference+i] = getFloatOfElement(value, ':', startpos, i);
			}
		}
	}
	else if(tempS20.Contains("defaultToolLength=") == 0) {
		int ct = getValueCount(value, "defaultToolLength=", ':', startpos);
		if(ct == 1) {
			currentToolLength = getFloatOfElement(value, ':', startpos, 0);
		}
	}
	else if(tempS20.Contains("toolDirection=") == 0) {
		int ct = getValueCount(value, "toolDirection=", ':', startpos);
		if(ct == 3) {
			toolDirection[0] = getFloatOfElement(value, ':', startpos, 0);
			toolDirection[1] = getFloatOfElement(value, ':', startpos, 1);
			toolDirection[2] = getFloatOfElement(value, ':', startpos, 2);
			normalizeVector(toolDirection[0], toolDirection[1], toolDirection[2]);
		}
	}
	else { // letter
		// C"letter=omega1:omega2:omega3:q1:q2:q3" axis orientation and a point on the axis
		char letter = value[0];
		int pos = getPositionOfLetterInChain(letter);

		char mark[3] = {letter, '=', '\0'};
		int ct = getValueCount(value, mark, ':', startpos);

		if(ct == 6) {
			cache[offsetScrewOmega + pos*3] = getFloatOfElement(value, ':', startpos, 0);
			cache[offsetScrewOmega + pos*3 + 1] = getFloatOfElement(value, ':', startpos, 1);
			cache[offsetScrewOmega + pos*3 + 2] = getFloatOfElement(value, ':', startpos, 2);
			normalizeVector(cache[offsetScrewOmega + pos*3], cache[offsetScrewOmega + pos*3 + 1],
					cache[offsetScrewOmega + pos*3 + 2]);

			cache[offsetScrewQ + pos*3] = getFloatOfElement(value, ':', startpos, 3);
			cache[offsetScrewQ + pos*3 + 1] = getFloatOfElement(value, ':', startpos, 4);
			cache[offsetScrewQ + pos*3 + 2] = getFloatOfElement(value, ':', startpos, 5);

			if(axisTypes[pos] == 'R') {
				int rotaryNr = getRotaryIndex(pos); // index of R axes. Analyses axisTypes
				if(rotaryNr > -1) {
					setRodrigues1(&cache[pos*3 + offsetScrewOmega], &cache[pos*3 + offsetScrewQ],
							&cache[rotaryNr*12 + offsetScrewOmega2], &cache[rotaryNr*3 + offsetScrewV]);
				}
			}

		}

	}
}

/*
 * get number of elements which are separated by separator (: in most cases)
 *
 * \param startpos is the position after the = (length of marker incl =)
 * \return the count is without the beginning text= marker
 */
// tested
int RobotKinematics::getValueCount(const char *value, const char *marker, char separator, int &startpos)
	const noexcept {

	int lenMarker = 0;
	for(int i=0; i < 500; i++) {
		if(marker[i]== '\0') {
			break;
		}
		lenMarker++;
	}
	startpos = lenMarker;

	int ct = 0;
	for(int i=0; i< 500; i++) {
		if(value[i]== '\0') {
			break;
		}
		else if(value[i] == separator) {
			ct++;
		}
	}
	return ct + 1;
}

void RobotKinematics::setA(const char *value) const noexcept {
	char letter = value[0];
	int chainOrder = getPositionOfLetterInChain(letter);
	char marker[3] = {letter, '=', '\0'};
	int startpos;
	int ct = getValueCount(value, marker, ':', startpos);
	if(ct == 3) {
		cache[offsetAngleLimits + chainOrder*3] = getFloatOfElement(value, ':', startpos, 0); // min
		cache[offsetAngleLimits + chainOrder*3 + 1] = getFloatOfElement(value, ':', startpos, 1); // max
		cache[offsetAngleLimits + chainOrder*3 + 2] = getFloatOfElement(value, ':', startpos, 2); // home
	}
}

int RobotKinematics::getPositionOfLetterInChain(char letter) const noexcept {
	int dotstart = -1;
	// letter of normal kin
	for(int i=0; i < numOfAxes; i++) {
		if(forwardChainCompressed[i] == letter) {
			return i;
		}
		if(forwardChainCompressed[i] == '.' && dotstart == -1) {
			dotstart = i;
		}
	}
	// special kin
	for(int i=0; i < 5; i++) {
		if(forwardChainSpecial[i] == '\0') {
			break;
		}
		else if(forwardChainSpecial[i] == letter) {
			if(dotstart != -1) {
				return dotstart + i;
			}
		}
	}
	return -1; // not found
}

void RobotKinematics::setP(const char *value) const noexcept {
	tempS20.copy(value);
	if(tempS20.Contains("axisTypes=") == 0) {
		int pos = 0;
		for(int i=10; i < 50; i++) {
			if(value[i] == '\0') {
				axisTypes[pos] = '\0';
				break;
			}
			axisTypes[pos] = value[i];
			pos++;
		}
		setNumOfAxes();
	}
	else {

	}

}

/*
 * \return letter. If special, look into forwardChainSpecial. \0 if not found
 */

char RobotKinematics::getLetterInChain(int pos) const noexcept {
	if(pos >= numOfAxes || pos < 0) {
		return '\0';
	}
	char letter = forwardChainCompressed[pos];
	if(letter == '.') { // special method letter
		int ctBefore = 0;
		for(int i=0; i < numOfAxes; i++) {
			if(forwardChainCompressed[i] == '.' && i < pos) {
				ctBefore++;
			}
			else if(i >= pos) {
				break;
			}
		}
		letter = forwardChainSpecial[ctBefore];
	}
	return letter;
}

void RobotKinematics::startClock() const noexcept {
	starttime = StepTimer::GetTimerTicks();
}

/*
 * return microseconds
 */

float RobotKinematics::stopClock() const noexcept {
	uint32_t stoptime = StepTimer::GetTimerTicks();
	uint32_t tickrate = StepTimer::GetTickRate(); // 750000 => 750 kHz => 1.33 microseconds for one tick
	float microsecondsForOneTick = 1e6f / (float) tickrate;
	if(starttime < stoptime) {
		return (float) (stoptime - starttime) * microsecondsForOneTick;
	}
	else {
		return 0.0; // todo handle overflow
	}
}


void RobotKinematics::checkAndChangeToolLength() const noexcept {
	float oldToolLength = currentToolLength;

	// todo support all toolDirections

	float newOffsets[3];
	getToolOffsets(newOffsets);

	if(toolDirection[0] == 0.0 && toolDirection[1] == 0.0 && toolDirection[2] == 1.0) {
		if(oldToolLength != newOffsets[2]) {
			float longer = newOffsets[2] - oldToolLength;
			screw_M[11] -= longer;
			recalcMInv();
			currentToolLength = newOffsets[2];
		}
	}
	else if(toolDirection[0] == 1.0 && toolDirection[1] == 0.0 && toolDirection[2] == 0.0) {
		if(oldToolLength != newOffsets[0]) {
			float longer = newOffsets[2] - oldToolLength;
			screw_M[3] -= longer;
			recalcMInv();
			currentToolLength = newOffsets[0];
		}
	}
	else if(toolDirection[0] == 0.0 && toolDirection[1] == 1.0 && toolDirection[2] == 0.0) {
		if(oldToolLength != newOffsets[1]) {
			float longer = newOffsets[2] - oldToolLength;
			screw_M[7] -= longer;
			recalcMInv();
			currentToolLength = newOffsets[1];
		}
	}
}

#ifndef _NCAP_COLLISION_OBSERVATION_HPP_
#define _NCAP_COLLISION_OBSERVATION_HPP_
#include <oppt/robotHeaders/Observation.hpp>
#include "NCAPDefines/NCAPCollisionGeneralUtils.hpp"
#include <limits>

using std::cout;
using std::endl;


namespace oppt {
class NCAPCollisionObservation: public VectorObservation {
public:
	NCAPCollisionObservation(VectorFloat& observationVec):
		VectorObservation(observationVec) {

	}


	virtual ~NCAPCollisionObservation() {}

	virtual FloatType distanceTo(const Observation& otherObservation) const override {
		// Very high number to make it a different observation by default
		FloatType dist = 100000; 
		const FloatType LONGIT_OFFSET = 5;
		const FloatType HORIZONTAL_OFFSET = 1.75;
		const FloatType LONGIT_DISCRETE = 3.125;
		const FloatType HORIZONTAL_DISCRETE = 1.5;
		VectorFloat otherObsVec = static_cast<const NCAPCollisionObservation &>(otherObservation).asVector();
		FloatType thisObsBinLongit = std::ceil(
			(observationVec_[OBSERVATION_INFO::REL_LONGIT] + LONGIT_OFFSET) // Offset to positive bins
			 	/ LONGIT_DISCRETE);

		FloatType thisObsBinHorizontal = std::ceil(
			(observationVec_[OBSERVATION_INFO::REL_HORIZONTAL] + HORIZONTAL_OFFSET) // Offset to positive bins
			 	/ HORIZONTAL_DISCRETE);

		// Comparison observation
		FloatType otherBinLongit = std::ceil(
			(otherObsVec[OBSERVATION_INFO::REL_LONGIT] + LONGIT_OFFSET)
				 / LONGIT_DISCRETE);

		FloatType otherBinHorizontal = std::ceil(
			(otherObsVec[OBSERVATION_INFO::REL_HORIZONTAL] + HORIZONTAL_OFFSET)
				/ HORIZONTAL_DISCRETE);


		// Add the L2-distance between observations
 		if((thisObsBinLongit == otherBinLongit) && (thisObsBinHorizontal == otherBinHorizontal)){
			// Same discretized bin is observed. Consider same observation
			dist = 0;
		}


		//Return big distance for different observations (bins). Otherwise, observation is the same
		return dist;
	}


};
}

#endif
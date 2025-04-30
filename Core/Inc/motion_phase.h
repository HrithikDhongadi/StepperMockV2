/*
 * motion_phase.h
 *
 *  Created on: Apr 30, 2025
 *      Author: hrith
 */

#ifndef INC_MOTION_PHASE_H_
#define INC_MOTION_PHASE_H_

typedef enum {
  MOTION_IDLE,
  MOTION_ACCELERATING,
  MOTION_CONSTANT,
  MOTION_DECELERATING
} MotionPhase;


#endif /* INC_MOTION_PHASE_H_ */

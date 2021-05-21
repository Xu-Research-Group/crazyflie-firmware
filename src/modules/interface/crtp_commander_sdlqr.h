/**
 * Authored by Victor Freire (https://xu.me.wisc.edu/), May 2021
 *
 * Header file for State Dependent LQR receiver of new Kalman Gain matrices K
 *
 * The packet format is:
 * +-----+========================+
 * | ROW |    DATA                |
 * +-----+========================+
 *
 * ROW contains the row uint8_t (0,1,2,3) to be updated of the matrix K. The data
 * contains the 9 entries of the specified row as a compressed (x1000.0f)
 * type int16_t. The total packet size should be 19 bytes
 *
 */

#ifndef __CRTP_COMMANDER_SDLQR_TASK_H__
#define __CRTP_COMMANDER_SDLQR_TASK_H__

#include <stdbool.h>

// Initialize and Test the task
void crtpCommanderSDLQRInit();
bool crtpCommanderSDLQRTest();

#endif // __CRTP_COMMANDER_SDLQR_TASK_H__

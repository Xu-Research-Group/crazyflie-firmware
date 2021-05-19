/**
 *   Authored by Victor Freire (https://xu.me.wisc.edu/), May 2021
 *
 *
 */

#include "config.h"
#include "debug.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "static_mem.h"
#include "task.h"

#include "crtp.h"
#include "crtp_commander_sdlqr.h"
#include "controller_lqr.h"

// One row of the K matrix (9 columns)
struct MatrixRowPacket_s{ // 18 bytes
  int16_t entries[9];
} __attribute__((packed));

// Global variables
static bool isInit = false;

// Private Functions
static void crtpCommanderSDLQRTask(void *parameters);
int handlePacket(const uint8_t row, const void *data, size_t datalen);

// Allocate memory for this task
STATIC_MEM_TASK_ALLOC(crtpCommanderSDLQRTask, CMD_SDLQR_TASK_STACKSIZE);

// Init function
void crtpCommanderSDLQRInit(){
    if (isInit){
        return;
    }

    //Start the SDLQR task
    STATIC_MEM_TASK_CREATE(crtpCommanderSDLQRTask, crtpCommanderSDLQRTask, CMD_SDLQR_TASK_NAME, NULL, CMD_SDLQR_TASK_PRI);

    // Initialized
    isInit = true;
}

// Test function
bool crtpCommanderSDLQRTest(){
    return isInit;
}

// Main task function
void crtpCommanderSDLQRTask(void * parameters){
    // Packets and init queue listener
    CRTPPacket pk;
    crtpInitTaskQueue(CRTP_PORT_SETPOINT_SDLQR);
    DEBUG_PRINT("COMMANDER: SDLQR Task is running\n"); // Signal ready
    while(true){ // Main loop: Process crtp packets in SDLQR port
        crtpReceivePacketBlock(CRTP_PORT_SETPOINT_SDLQR, &pk);
        handlePacket(pk.data[0], ((char*)pk.data)+1, pk.size-1);
    }
}

// Decoder switch for each row of matrix K. Returns the updated row
int handlePacket(const uint8_t row, const void *data, size_t datalen){
  DEBUG_PRINT("COMMANDER: SDLQR Packet received\n"); // Signal
  const struct MatrixRowPacket_s *K_row = data;
  ASSERT(datalen == sizeof(struct MatrixRowPacket_s));
  // Convert and Store in K
  for (int i=0; i<9; i++){
      update_K_entry(row,i,(float)K_row->entries[i]/1000.0f);
  }
  return row;
}



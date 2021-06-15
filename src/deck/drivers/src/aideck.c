/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * aideck.c - Deck driver for the AIdeck
 */
#define DEBUG_MODULE "AIDECK"

#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "stm32fxxx.h"
#include "config.h"
#include "console.h"
#include "uart1.h"
#include "debug.h"
#include "deck.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <stdlib.h>
#include <math.h>
#include "log.h"
#include "param.h"
#include "system.h"
#include "uart1.h"
#include "uart2.h"

#include "aideck.h"
#include "stabilizer_types.h"

#ifdef AI_CBF
static u_t u;
static CBFPacket pk_rx; // Packet for receiving via UART
static CBFPacket pk_tx; // Packet to send via UART
static uint8_t aideck_ready_flag; // Set to 1 whenever a CBFPacket is received via UART
static uint8_t missed_cycles; // Keeps track of the number of cbf_qp_data that have been discarded
static cbf_qp_data_comp_t data_comp; // Compressed CBF-QP Data
#endif
static bool isInit = false;
static char byte; // char buffer for RX

//Uncomment when NINA printout read is desired from console
//#define DEBUG_NINA_PRINT

#ifdef DEBUG_NINA_PRINT
static void NinaTask(void *param)
{
    systemWaitStart();
    vTaskDelay(M2T(1000));
    DEBUG_PRINT("Starting reading out NINA debugging messages:\n");
    vTaskDelay(M2T(2000));

    // Pull the reset button to get a clean read out of the data
    pinMode(DECK_GPIO_IO4, OUTPUT);
    digitalWrite(DECK_GPIO_IO4, LOW);
    vTaskDelay(10);
    digitalWrite(DECK_GPIO_IO4, HIGH);
    pinMode(DECK_GPIO_IO4, INPUT_PULLUP);

    // Read out the byte the NINA sends and immediately send it to the console.
    uint8_t byte;
    while (1)
    {
        if (uart2GetDataWithDefaultTimeout(&byte) == true)
        {
            consolePutchar(byte);
        }
    }
}
#endif

#ifdef AI_CBF
// Update u with a stop command in case of error
static void force_stop_u(void){
  u.T = 0;
  u.p = 0;
  u.q = 0;
  u.r = 0;
}
#endif

#ifdef AI_CBF_DEBUG
// DEBUG PRINT the u_t struct
static void print_u(void){
  DEBUG_PRINT("u.T = %.4f\n",(double)u.T);
  DEBUG_PRINT("u.p = %.4f\n",(double)u.p);
  DEBUG_PRINT("u.q = %.4f\n",(double)u.q);
  DEBUG_PRINT("u.r = %.4f\n",(double)u.r);
  DEBUG_PRINT("Missed Cycles = %d\n", missed_cycles);
}
#endif

#ifdef AI_CBF
// Update the u struct from received data
static void unpack(void){
  pk_rx.header = 0;
  for(int i=0; i<sizeof(u_t); i++){
    ((uint8_t*)&u)[i] = pk_rx.data[i];
    pk_rx.data[i] = '\0'; // Empty packet
  }
}
#endif

#ifdef AI_CBF
// Receive a full CBFPacket via UART
// return 0 if healthy pk, 1 otherwise
static int receive_pk(void){
  uart1Getchar(&byte); // Receive byte
  pk_rx.header = byte; // Update header
  int unhealthy = (byte!='V'); // Is it healthy?
  if(unhealthy) return unhealthy;
  // Receive rest of the packet if healthy
  for(int i=0; i<MAX_CBFPACKET_DATA_SIZE; i++){
    uart1Getchar(&byte); // Receive byte
    pk_rx.data[i] = byte;
  }
  unpack(); // Unpack the CBFPacket
  aideck_ready_flag = 1; // AI Deck is ready for more data
  return unhealthy;
}
#endif


// AI Deck task to listen for UART traffic from the AI Deck
static void Gap8Task(void *param) {
  systemWaitStart();
  vTaskDelay(M2T(1000));

  // Pull the reset button to get a clean read out of the data
  pinMode(DECK_GPIO_IO4, OUTPUT);
  digitalWrite(DECK_GPIO_IO4, LOW);
  vTaskDelay(10);
  digitalWrite(DECK_GPIO_IO4, HIGH);
  pinMode(DECK_GPIO_IO4, INPUT_PULLUP);

  // Receive data in a loop
  while (1){
#ifdef AI_CBF
    if (receive_pk()){ // Flush RX after unhealthy packet
      pinMode(DECK_GPIO_IO4, OUTPUT); // TODO Is this needed?
      digitalWrite(DECK_GPIO_IO4, LOW);
      vTaskDelay(10);
      digitalWrite(DECK_GPIO_IO4, HIGH);
      pinMode(DECK_GPIO_IO4, INPUT_PULLUP);
    }
#else
    vTaskDelay(M2T(1000));
#endif
#ifdef AI_CBF_DEBUG
    else
      print_u();
#endif
  }
}

static void aideckInit(DeckInfo *info)
{

  if (isInit)
      return;

  // Intialize the UART for the GAP8
  uart1Init(115200);
  // Initialize task for the GAP8
  xTaskCreate(Gap8Task, AI_DECK_GAP_TASK_NAME, AI_DECK_TASK_STACKSIZE, NULL,
              AI_DECK_TASK_PRI, NULL);

#ifdef AI_CBF
  // The AI Deck is ready for UART Data
  aideck_ready_flag = 1;
#endif

#ifdef DEBUG_NINA_PRINT
  // Initialize the UART for the NINA
  uart2Init(115200);
  // Initialize task for the NINA
  xTaskCreate(NinaTask, AI_DECK_NINA_TASK_NAME, AI_DECK_TASK_STACKSIZE, NULL,
                AI_DECK_TASK_PRI, NULL);
#endif

  isInit = true;
}

static bool aideckTest()
{

    return true;
}

#ifdef AI_CBF
// Send the CBF-QP Parametric data via UART1
void aideck_send_cbf_data(const cbf_qp_data_t *data){
  if(aideck_ready_flag){ // Is the AI Deck ready to receive data?
    // Compress data
    data_comp.phi = (int16_t)(data->phi*1000.0f);
    data_comp.theta = (int16_t)(data->theta*1000.0f);
    data_comp.u.T = (int16_t)(data->u.T*1000.0f);
    data_comp.u.p = (int16_t)(data->u.p*1000.0f);
    data_comp.u.q = (int16_t)(data->u.q*1000.0f);
    data_comp.u.r = (int16_t)(data->u.r*1000.0f);
    // Pack data
    cbf_pack(sizeof(cbf_qp_data_comp_t), (uint8_t *)&data_comp);
    // Send packet
    uart1SendData(sizeof(CBFPacket), (void *)pk_tx.raw);
    // AI Deck is processing the data
    aideck_ready_flag = 0;
    missed_cycles = 0; // Reset cycles
  }
  else{
    missed_cycles++; // Missed this cycle
#ifdef AI_CBF_DEBUG
    //DEBUG_PRINT("Missed Cycles = %d\n",missed_cycles);
#endif
    if(missed_cycles > 200){ // Don't miss more than X cycles
      force_stop_u();
      aideck_ready_flag = 1; // Force ready
    }
  }
}
#endif

#ifdef AI_CBF
// Pack data into CBFPacket pk_tx
CBFPacket *cbf_pack(const uint8_t size, uint8_t *data){
  // Check data size
  if (size > MAX_CBFPACKET_DATA_SIZE){
    pk_tx.header = 0;
    return NULL;
  }
  // Populate header with 'V' char
  pk_tx.header = 'V'; // 86. 0x56
  // Fill data
  for(int i=0; i<MAX_CBFPACKET_DATA_SIZE; i++){
    if (i<size)
      pk_tx.data[i] = data[i];
    else
      pk_tx.data[i] = '\0';
  }
  return &pk_tx;
}
#endif

#ifdef AI_CBF
// Give controller the CBF-QP Solution
void aideck_get_safe_u(float *T, attitude_t *att){
  *T = u.T;
  att->roll = u.p;
  att->pitch = u.q;
  att->yaw = u.r;
}
#endif


static const DeckDriver aideck_deck = {
    .vid = 0xBC,
    .pid = 0x12,
    .name = "bcAI",

    .usedPeriph = 0,
    .usedGpio = 0, // FIXME: Edit the used GPIOs

    .init = aideckInit,
    .test = aideckTest,
};

LOG_GROUP_START(aideck)
LOG_ADD(LOG_UINT8, receivebyte, &byte)
LOG_GROUP_STOP(aideck)

/** @addtogroup deck
*/
PARAM_GROUP_START(deck)

/**
 * @brief Nonzero if [AI deck](https://store.bitcraze.io/collections/decks/products/ai-deck-1-1) is attached
 */
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcAIDeck, &isInit)

PARAM_GROUP_STOP(deck)

DECK_DRIVER(aideck_deck);

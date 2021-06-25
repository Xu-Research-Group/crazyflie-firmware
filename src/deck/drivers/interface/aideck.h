/**
 * Authored by Victor Freire (https://xu.me.wisc.edu/), June 2021
 *
 *
 *
 */

#ifndef _AIDECK_H_
#define _AIDECK_H_

#include "stabilizer_types.h"
#include "controller_lqr.h"

#define MAX_CBFPACKET_DATA_SIZE 16

/**
 * UART Packet structure (MAX_CBF_PACKET_DATA_SIZE+1)Bytes
 * Header:
 * 'ARC' = healthy packet
 * 'LAB' = stop packet
 */
typedef union{
  struct{
    uint8_t header; // 'V' or 'X'
    uint8_t data[MAX_CBFPACKET_DATA_SIZE];
  };
  uint8_t raw[MAX_CBFPACKET_DATA_SIZE+1];
} __attribute__((packed)) CBFPacket;

/** Control Input to be sent via UART to Crazyflie Firmware 16Bytes */
typedef union u_s{
  struct{
    float T;  // Normalized thrust [m/s^2]
    float p;  // Angular velocities [rad/s]
    float q;
    float r;
  };
  float arr[4];
} __attribute__((packed)) u_t;

/** Compressed version of u_s 8Bytes*/
typedef union u_comp_s{
  struct{
    int16_t T;  // Normalized thrust [mm/s^2]
    int16_t p;  // Angular velocities [milirad/s]
    int16_t q;
    int16_t r;
  };
  int16_t arr[4];
} __attribute__((packed)) u_comp_t;


/** OSQP Parametric data to be updated in the OSQPData struct 24Bytes */
typedef union cbf_qp_data_s{
  struct{
    float phi;   // Roll  [rad]
    float theta; // Pitch [rad]
    u_t u;       // Nominal input to adjust
  };
  float arr[6];
} cbf_qp_data_t;

/** Compressed version of cbf_qp_data_s 12Bytes */
typedef union cbf_qp_data_comp_s{
  struct{
    int16_t phi;   // Roll  [milirad]
    int16_t theta; // Pitch [milirad]
    u_comp_t u;     // Compressed Nominal input to adjust
  };
  int16_t arr[6];
} __attribute__((packed)) cbf_qp_data_comp_t;


/**
 * Pack size bytes of data into a CBFPacket
 *
 * returns a pointer to the CBFPacket if successful
 *         NULL if size of data is too large
 */
CBFPacket *cbf_pack(const uint8_t size, uint8_t *data);

/**
 * Send the CBF-QP parametric data to the AI Deck to update and solve the problem
 */
void aideck_send_cbf_data(const cbf_qp_data_t *data);

/**
 * Update controller variables with most recently received
 * solution to CBF-QP
 */
void aideck_get_safe_u(float u[4]);


#endif /* _AIDECK_H_ */

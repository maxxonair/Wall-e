#ifndef ROVER_RADIO_H

#define ROVER_RADIO_H

//------------------------------------------------------------------------------
// >> Radio Communication
//------------------------------------------------------------------------------
int CE  = 4;
int CSN = 5;
RF24 radio(CE,CSN);
const uint64_t NRF24_PIPE_ADR = 0xE8E8F0F0E1LL;

#endif
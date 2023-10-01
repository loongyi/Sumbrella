//Sumbrella functional fashion controller for dynamixel motors
//Programmed on an MKR Zero and OPENRB-150

/*
=======SUMMARY=========
Takes serial message - motor (three modes -L,R,all), frequency(1-10), Distance(continuous),
as an array- (e.g., a3 will lead to motor a rotating at 30% max frequency)and runs the dynamixels 
*/
//variable affected by serial message -

//dependencies
#include <DynamixelShield.h>

//Definitions
//serial
#if defined(ARDUINO_OpenRB)  // When using OpenRB-150 \
                             //OpenRB does not require the DIR control pin.
#define DXL_SERIAL Serial1
#define DEBUG_SERIAL Serial
#else
#define DEBUG_SERIAL Serial
#endif

const uint8_t BROADCAST_ID = 254;
const float DYNAMIXEL_PROTOCOL_VERSION = 2.0;
const uint8_t DXL_ID_CNT = 3;
const uint8_t DXL_ID_LIST[DXL_ID_CNT] = { 1, 2, 3 };
const uint16_t user_pkt_buf_cap = 128;
uint8_t user_pkt_buf[user_pkt_buf_cap];

// Starting address of the Data to read; Present Current = 126; Present Position =132
const uint16_t SR_START_ADDR = 126;
// Length of the Data to read.
// Read 10 byte from 126 to 135
// Present Current: 2Byte, Present Velocity: 4Byte, Present Position: 4Byte
const uint16_t SR_ADDR_LEN = 10;
// Starting address of the Data to write; Goal Velocity = 104, Goal Position=116
const uint16_t SW_START_ADDR = 104;
const uint16_t SW_START_ADDRp = 116;
// Length of the Data to write; Length of Goal Velocity data of X series is 4 byte
const uint16_t SW_ADDR_LEN = 4;
typedef struct sr_data {
  int16_t present_current;
  int32_t present_velocity;
  int32_t present_position;
} __attribute__((packed)) sr_data_t;
typedef struct sw_data {
  int32_t goal;
} __attribute__((packed)) sw_data_t;

sr_data_t sr_data[DXL_ID_CNT];
DYNAMIXEL::InfoSyncReadInst_t sr_infos;
DYNAMIXEL::XELInfoSyncRead_t info_xels_sr[DXL_ID_CNT];

sw_data_t sw_data[DXL_ID_CNT];
DYNAMIXEL::InfoSyncWriteInst_t sw_infos;
DYNAMIXEL::XELInfoSyncWrite_t info_xels_sw[DXL_ID_CNT];

DynamixelShield dxl;
using namespace ControlTableItem;

//Main Variables


void setup() {
  // put your setup code here, to run once:
  uint8_t i;
  pinMode(LED_BUILTIN, OUTPUT);
  DEBUG_SERIAL.begin(115200);
  dxl.begin(57600);
  dxl.setPortProtocolVersion(DYNAMIXEL_PROTOCOL_VERSION);

  //default velocity mode
  for (i = 0; i < DXL_ID_CNT; i++) {
    dxl.torqueOff(DXL_ID_LIST[i]);
    delay(10);
    dxl.setOperatingMode(DXL_ID_LIST[i], OP_VELOCITY);
    //dxl.torqueOn(DXL_ID_LIST[i]);
  }
  dxl.torqueOn(BROADCAST_ID);

  // Fill the members of structure to syncRead using external user packet buffer
  sr_infos.packet.p_buf = user_pkt_buf;
  sr_infos.packet.buf_capacity = user_pkt_buf_cap;
  sr_infos.packet.is_completed = false;
  sr_infos.addr = SR_START_ADDR;
  sr_infos.addr_length = SR_ADDR_LEN;
  sr_infos.p_xels = info_xels_sr;
  sr_infos.xel_count = 0;

  // Prepare the SyncRead structure
  for (i = 0; i < DXL_ID_CNT; i++) {
    info_xels_sr[i].id = DXL_ID_LIST[i];
    info_xels_sr[i].p_recv_buf = (uint8_t*)&sr_data[i];
    sr_infos.xel_count++;
  }
  sr_infos.is_info_changed = true;

  // Fill the members of structure to syncWrite using internal packet buffer
  sw_infos.packet.p_buf = nullptr;
  sw_infos.packet.is_completed = false;
  sw_infos.addr = SW_START_ADDR;
  sw_infos.addr_length = SW_ADDR_LEN;
  sw_infos.p_xels = info_xels_sw;
  sw_infos.xel_count = 0;

  sw_data[0].goal = 0;
  sw_data[1].goal = 0;
  sw_data[2].goal = 0;

  for (i = 0; i < DXL_ID_CNT; i++) {
    info_xels_sw[i].id = DXL_ID_LIST[i];
    info_xels_sw[i].p_data = (uint8_t*)&sw_data[i].goal;
    sw_infos.xel_count++;
  }
  sw_infos.is_info_changed = true;
}

int goals[3];  // functions will change these values, which dictate motor motion.

double posINT_m1 = dxl.getPresentPosition(1);
double posINT_m2 = dxl.getPresentPosition(2);
double posINT_m3 = dxl.getPresentPosition(3);
double posStroke_m1 = 6000;
double posStroke_m2 = 6000;
double posStroke_m3 = 6000;

//motor enable flags
int flag_m1 = 0;
int flag_m2 = 0;
int flag_m3 = 0;

//down flags
int flag_m1d = 0;
int flag_m2d = 0;
int flag_m3d = 0;

//motor activation states // 0 = pull up, 1 = release1
int m1state = 100;
int m2state = 100;
int m3state = 100;

//displacement residuals during stroke
double m1disres = 0;
double m2disres = 0;
double m3disres = 0;

int dist = 0;
int state = 0;
int prev_state=0;
int msg = 0, rejectedmsg=0;

static float time = 0.0;

void loop() {

  if ( Serial.available() && (flag_m1+flag_m2+flag_m3==0) ) {
    //command from RPI received
    //arduino receives serial message when pi +camera detects person

    msg = Serial.readStringUntil('\n').toInt();

    //extract dsitance and state from  received integer
    state = msg % 10;
    dist = (msg / 10) % 10;
    DEBUG_SERIAL.print("DIST: ");
    DEBUG_SERIAL.print(dist);
    DEBUG_SERIAL.print("  STATE: ");
    DEBUG_SERIAL.println(state);

    //motor enable flags, serviced by motor routines
    Serial.flush();
  }
  else if (Serial.available() && flag_m1+flag_m2+flag_m3>0){
   rejectedmsg = Serial.readStringUntil('\n').toInt();
  }


  // DEBUG_SERIAL.print("FLAG  ");
  // DEBUG_SERIAL.print(flag_m1);

  // DEBUG_SERIAL.print("   STATE  ");
  // DEBUG_SERIAL.println(m1state);
  //int dir- 1 for pullup  for m1 and m3, -1, for pullup for m2 vice versa

  //what happens when it changes from 'lifting one' state to lifting all state? I think the lifted one just keeps going to end.3
  //but how do we ensure it can remember its initial position before switching states?

  //we are currently doubling up on the travel, if the robot goes up , goes down a bit, then goes up again, the robot cant get back to initial state
  switch (state) {
  case 0: //STOP motor //resets all flags
    goals[1]=0;flag_m1=0;m1disres=0;
    goals[2]=0;flag_m2=0;m2disres=0;
    goals[3]=0;flag_m3=0;m3disres=0;

  // case 1: // LIFT LEFT UP
  //   if (state != prev_state){ 
  //     flag_m1 = 1; 
  //     posINT_m1 = dxl.getPresentPosition(1);
  //   }
  //   if ((m1state==1)||(m1state==100)){m1state = 0;}
    
  //   break;

  case 1: // LIFT LEFT UP
    if (state != prev_state){ 
      flag_m1 = 1; flag_m2=1;
      posINT_m1 = dxl.getPresentPosition(1);
    }
    if ((m1state==1)||(m1state==100)){m1state = 0;}
    if ((m2state == 0)){m2state= 1;}
    break;

 // case 2: // LIFT RIGHT UP
    // if (state != prev_state){ 
    //   flag_m2 = 1;
    //   posINT_m2 = dxl.getPresentPosition(2);
    // }
    // if ((m2state==1)||(m2state==100)){m2state = 0;}
    // if ((m1state == 0)){m1state= 1;}
    // break;

  case 2: // LIFT RIGHT UP
    if (state != prev_state){ 
      flag_m2 = 1;flag_m1=1;
      posINT_m2 = dxl.getPresentPosition(2);
    }
    if ((m2state==1)||(m2state==100)){m2state = 0;}
    if ((m1state == 0)){m1state= 1;}
    break;

  case 3: //RELEASE LEFT DOWN
    if ((state != prev_state)){ 
      flag_m1 = 1;
      posINT_m1 = dxl.getPresentPosition(1);
    }
    if (m1state==0){m1state = 1;}
    break;

  case 4://RELEASE RGHT DOWN 
    if (state != prev_state){ 
      flag_m2 = 1; 
      posINT_m2 = dxl.getPresentPosition(2);
    }
    if (m2state==0){m2state = 1;}
    break;

  case 5://Lift back up
    if (state != prev_state){ 
      flag_m3 = 1; 
      posINT_m3 = dxl.getPresentPosition(3);
    }
    m3state = 0;
    break;

  case 6://release back down
    if (state != prev_state){ 
      flag_m3 = 1;
      posINT_m3 = dxl.getPresentPosition(3);
    }
    m3state = 1;
    break;

  case 7://all pull up
     if (state != prev_state){ 
      flag_m3 = 1; flag_m2 = 1; flag_m1 = 1;
      posINT_m1 = dxl.getPresentPosition(1);
      posINT_m2 = dxl.getPresentPosition(2);
      posINT_m3 = dxl.getPresentPosition(3);
     }
     m3state = 0; m2state = 0; m1state = 0;
    break;

  case 8://all release
     if (state != prev_state){ 
      flag_m3 = 1; flag_m2 = 1; flag_m1 = 1;
      posINT_m1 = dxl.getPresentPosition(1);
      posINT_m2 = dxl.getPresentPosition(2);
      posINT_m3 = dxl.getPresentPosition(3);
     }
     m3state = 1; m2state = 1; m1state = 1;    
    break;

  case 9://all wave
    if (state != prev_state){ 
      time=0; 

    }
    wave(2);
    break;
  }

    // DEBUG_SERIAL.print(prev_state);
    // DEBUG_SERIAL.print("  ");
    // DEBUG_SERIAL.print(flag_m1);
    // DEBUG_SERIAL.print("  ");
    // DEBUG_SERIAL.print(posINT_m1);
    // DEBUG_SERIAL.print("  ");
    // DEBUG_SERIAL.println(posStroke_m1);

  if((state!=0)&&(state!=9)){ 
    pullreleasevel1(m1state, posINT_m1, -1);
    pullreleasevel2(m2state, posINT_m2, 1);
    pullreleasevel3(m3state, posINT_m3, -1);
  }

  //write them to the motors
  SYNCW();
  prev_state = state;

  // if (prev_state==9){
  //   //       DEBUG_SERIAL.print("   ");
  //   //DEBUG_SERIAL.println(flag_m1);
  //   if (flag_m1==0){
  //   goals[1]=0;}
  //   if (flag_m2==0){
  //   goals[2]=0;}
  //   if (flag_m3==0){
  //   goals[3]=0;}
  // }
  //overcurent protection

     if (abs(dxl.getPresentCurrent(1, UNIT_PERCENT))>66){
        goals[1] = 0; flag_m1 = 0;
     }
     if (abs(dxl.getPresentCurrent(2, UNIT_PERCENT))>66){
        goals[2] = 0; flag_m2 = 0;
     }
     if (abs(dxl.getPresentCurrent(3, UNIT_PERCENT))>66){
        goals[3] = 0; flag_m3 = 0;
     }
}

void SYNCW() {

  sw_data[0].goal = goals[1];  //1 = left
  sw_data[1].goal = goals[2];  //2 = right
  sw_data[2].goal = goals[3];  //3 = back
  sw_infos.is_info_changed = true;

  dxl.syncWrite(&sw_infos);
  //digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  delay(10);
}
void SYNCR() {
  // Transmit predefined SyncRead instruction packet
  // and receive a status packet from each DYNAMIXEL
  uint8_t i, recv_cnt;
  recv_cnt = dxl.syncRead(&sr_infos);
  if (recv_cnt > 0) {
    DEBUG_SERIAL.print("[SyncRead] Success, Received ID Count: ");
    DEBUG_SERIAL.println(recv_cnt);

    for (i = 0; i < recv_cnt; i++) {
      DEBUG_SERIAL.print("  ID: ");
      DEBUG_SERIAL.print(sr_infos.p_xels[i].id);
      DEBUG_SERIAL.print(", Error: ");
      DEBUG_SERIAL.println(sr_infos.p_xels[i].error);
      DEBUG_SERIAL.print("\t Present Current: ");
      DEBUG_SERIAL.println(sr_data[i].present_current);
      DEBUG_SERIAL.print("\t Present Velocity: ");
      DEBUG_SERIAL.println(sr_data[i].present_velocity);
      DEBUG_SERIAL.print("\t Present Position: ");
      DEBUG_SERIAL.println(sr_data[i].present_position);
    }
  } else {
    DEBUG_SERIAL.print("[SyncRead] Fail, Lib error code: ");
    DEBUG_SERIAL.println(dxl.getLastLibErrCode());
  }
  delay(10);
}

//int testcount=0;
//int testcount2=0;
//int testcount3=0;

int pullreleasevel1(int mstate, double posINIT, int dir) {
  int mo = 1;
  double posEND = dxl.getPresentPosition(mo);
  m1state = mstate;

  if (mstate == 0) {  //pullup
    if (abs(dxl.getPresentCurrent(mo, UNIT_PERCENT)) > 50) {
      goals[mo] = 0;
      flag_m1 = 0;
      posStroke_m1 = abs(posINIT - posEND)+m1disres;
    }
    if (flag_m1 == 1) {
      //if(testcount>30){flag_m1=0;testcount=0;}
      //testcount+=1;
      goals[mo] = 400 * dir;
      posStroke_m1 = abs(posINIT - posEND)+m1disres;
      //DEBUG_SERIAL.print("Pull: RUN   ");
    } else {
      //DEBUG_SERIAL.print("Pull: STOP   ");
    }
  } 
  else if (mstate == 1) {  //release
    flag_m1d = 1;
    m1disres=abs(posINIT - posEND);
    if (abs(dxl.getPresentCurrent(mo, UNIT_PERCENT))>50){
        goals[mo] = 0;
        flag_m1 = 0;
        //posStroke_m1 = 0;
    }
    if (flag_m1 == 1) {
      goals[mo] = -40 * dir;
      //DEBUG_SERIAL.print("Rel: RUN   ");
    } else {
      //DEBUG_SERIAL.print("Rel: STOP   ");
    }
    if (abs(posINIT - posEND) > posStroke_m1 * 0.999) {

      goals[mo] = 0;
      flag_m1 = 0;
      posStroke_m1 = 0;
      flag_m1d = 0;
      m1disres = 0;
    }
  }
  // DEBUG_SERIAL.print(abs(dxl.getPresentCurrent(mo, UNIT_PERCENT)));
  // DEBUG_SERIAL.print("   ");
  // DEBUG_SERIAL.print(posStroke_m1);
  // DEBUG_SERIAL.print("   ");
  // DEBUG_SERIAL.print(abs(posINIT-posEND));
  //   DEBUG_SERIAL.print("   ");
  // DEBUG_SERIAL.println(abs(posINIT));
}
int pullreleasevel2(int mstate, double posINIT, int dir) {
  int mo = 2;

  double posEND = dxl.getPresentPosition(mo);
  m2state = mstate;

  if (mstate == 0) {  //pullup

    if (abs(dxl.getPresentCurrent(mo, UNIT_PERCENT)) > 50) {
      goals[mo] = 0;
      flag_m2 = 0;
      posStroke_m2 = abs(posINIT - posEND)+m2disres;
      
    }
    if (flag_m2 == 1) {
      //if(testcount2>30){flag_m2=0;testcount2=0;}
      //testcount2+=1;

      goals[mo] = 400 * dir;
      posStroke_m2 = abs(posINIT - posEND)+m2disres;
      //DEBUG_SERIAL.print("Pull: RUN   ");
    } else {
      //DEBUG_SERIAL.print("Pull: STOP   ");
    }
  } 
  else if (mstate == 1) {  //release
    flag_m2d = 1;
    m2disres=(posStroke_m2 - abs(posINIT - posEND));
    if (abs(dxl.getPresentCurrent(mo, UNIT_PERCENT))>50){
        goals[mo] = 0;
        flag_m2 = 0;
        //posStroke_m2 = 0;      
    }
    if (flag_m2 == 1) {
      
      goals[mo] = -40 * dir;
      //DEBUG_SERIAL.print("Rel: RUN   ");

    } else {
      //DEBUG_SERIAL.print("Rel: STOP   ");
    }
    if (abs(posINIT - posEND) > posStroke_m2 * 0.999) {
      goals[mo] = 0;
      flag_m2 = 0;
      posStroke_m2 = 0;
      flag_m2d = 0;
      m2disres = 0;
    }
  }
  //  DEBUG_SERIAL.print(flag_m2);
  //  DEBUG_SERIAL.print("   ");
  //  DEBUG_SERIAL.print(abs(dxl.getPresentCurrent(mo, UNIT_PERCENT)));
  //  DEBUG_SERIAL.print("   ");
  //  DEBUG_SERIAL.print(posStroke_m2);
  //  DEBUG_SERIAL.print("   ");
  //  DEBUG_SERIAL.print(abs(posINIT - posEND));
  //  DEBUG_SERIAL.print("   ");
  //  DEBUG_SERIAL.println(posINIT);
  //  DEBUG_SERIAL.print("   ");
  //  DEBUG_SERIAL.println(posEND);
  //  DEBUG_SERIAL.print("   ");
  //  DEBUG_SERIAL.println(m2disres);
}
int pullreleasevel3(int mstate, double posINIT, int dir) {
   int mo = 3;

  double posEND = dxl.getPresentPosition(mo);
  m3state = mstate;

  if (mstate == 0) {  //pullup

    if (abs(dxl.getPresentCurrent(mo, UNIT_PERCENT)) > 50) {
      goals[mo] = 0;
      flag_m3 = 0;
      posStroke_m3 = abs(posINIT - posEND)+m3disres;
      
    }
    if (flag_m3 == 1) {
      //if(testcount3>30){flag_m3=0;testcount3=0;}
      //testcount3+=1;
      goals[mo] = 400 * dir;
      posStroke_m3 = abs(posINIT - posEND)+m3disres;
      //DEBUG_SERIAL.print("Pull: RUN   ");
    } else {
      //DEBUG_SERIAL.print("Pull: STOP   ");
    }
  } 
  else if (mstate == 1) {  //release
    flag_m3d = 1;
    m3disres=(posStroke_m3 - abs(posINIT - posEND));
    if (abs(dxl.getPresentCurrent(mo, UNIT_PERCENT))>50){
        goals[mo] = 0;
        flag_m3 = 0;
        //posStroke_m3 = 0;      
    }
    if (flag_m3 == 1) {
      
      goals[mo] = -40 * dir;
      //DEBUG_SERIAL.print("Rel: RUN   ");

    } else {
      //DEBUG_SERIAL.print("Rel: STOP   ");
    }
    if (abs(posINIT - posEND) > posStroke_m3 * 0.99) {
      goals[mo] = 0;
      flag_m3 = 0;
      posStroke_m3 = 0;
      flag_m3d = 0;
      m3disres = 0;
    }
}
}

//wave paramters
#define AMPLITUDE 2048 // Amplitude of the sine wave every 2048 is 180 degrees
//#define FREQUENCY 2.0     // Frequency of the sine wave (Hz)
#define PHASE_SHIFT1 0.0  // Phase shift of the sine wave (radians)
#define PHASE_SHIFT2 PI/2 // Phase shift of the sine wave (radians)
#define PHASE_SHIFT3 2*PI/3 // Phase shift of the sine wave (radians)
#define TIME_STEP 0.01    // Time step for each iteration (seconds)
#define Kp 0.6

double thetap1 = 0; // present position in radial space
double thetap2= 0; // present position in radial space
double thetap3 = 0; // present position in radial space

float thetas1 = 0; //target position in radial space
float thetas2 = 0; //target position in radial space
float thetas3 = 0; //target position in radial space

float error1 = 0;
float error2 = 0;
float error3 = 0;

float act1 = 0;
float act2 = 0;
float act3 = 0;

float Pi1 = 0;
float Pi2 = 0;
float Pi3 = 0;

float calculateSinusoidalAngle(float amplitude, float frequency, float phaseShift, float time) {
  return amplitude * sin(2 * PI * frequency * time + phaseShift);
}

void wave(float FREQUENCY) {

  if (time < TIME_STEP){
    goals[1]=-50;
    goals[2]=50;
    goals[3]=-50;
    SYNCW();
    delay(1000);
    Pi1 = dxl.getPresentPosition(1);
    Pi2 = dxl.getPresentPosition(2);
    Pi3 = dxl.getPresentPosition(3);

  }

  thetas1 = Pi1+calculateSinusoidalAngle(AMPLITUDE, FREQUENCY, PHASE_SHIFT1, time);
  thetas2 = Pi2+calculateSinusoidalAngle(AMPLITUDE, FREQUENCY, PHASE_SHIFT1, time + PI);
  thetas3 = Pi3+calculateSinusoidalAngle(AMPLITUDE, FREQUENCY, PHASE_SHIFT1, time );

  thetap1 = dxl.getPresentPosition(1);
  thetap2 = dxl.getPresentPosition(2);
  thetap3 = dxl.getPresentPosition(3);

  error1 = thetas1-thetap1;
  error2 = thetas2-thetap2;
  error3 = thetas3-thetap3;

  int Vbase = 1;
  int blim=-100, ulim=100;
  
  act1 = Kp*error1;
  act2 = Kp*error2;
  act3 = Kp*error3;

  act1=constrain(act1,blim,ulim);
  act2=constrain(act2,blim,ulim);
  act3=constrain(act3,blim,ulim);

  act1 = map(act1,blim,ulim, Vbase*blim, Vbase*ulim);
  act2 = map(act2,blim,ulim, Vbase*blim, Vbase*ulim);
  act2 = map(act3,blim,ulim, Vbase*blim, Vbase*ulim);

  goals[1] = act1;
  goals[2] = -act2;
  goals[3] = act3;
  //goals[2] = map((int)motor2Angle, -AMPLITUDE, AMPLITUDE, 0, 4095);
  //goals[3] = map((int)motor2Angle, -AMPLITUDE, AMPLITUDE, 0, 4095);
  time += TIME_STEP;
    DEBUG_SERIAL.print(time);
    DEBUG_SERIAL.print("   ");
    DEBUG_SERIAL.print(Pi1);
    DEBUG_SERIAL.print("   ");
    DEBUG_SERIAL.print(act1);
    DEBUG_SERIAL.print("   ");
    DEBUG_SERIAL.print(thetap1);
    DEBUG_SERIAL.println("   ");
    DEBUG_SERIAL.print(thetas1);
    DEBUG_SERIAL.println("   ");
}

void modechange(String mode) {
  uint8_t i;
  for (i = 0; i < DXL_ID_CNT; i++) {
    dxl.torqueOff(DXL_ID_LIST[i]);
    delay(0.2);
    if (mode == "vel") {
      dxl.setOperatingMode(DXL_ID_LIST[i], OP_VELOCITY);
    }
    if (mode == "pos") {
      dxl.setOperatingMode(DXL_ID_LIST[i], OP_POSITION);
    }
  }
  dxl.torqueOn(BROADCAST_ID);
  //DEBUG_SERIAL.print("operating mode cahnged to ");
  //DEBUG_SERIAL.println(mode);
  delay(10);
}



  /* syncWrite
  Structures containing the necessary information to process the 'syncWrite' packet.

  typedef struct XELInfoSyncWrite{
    uint8_t* p_data;
    uint8_t id;
  } __attribute__((packed)) XELInfoSyncWrite_t;

  typedef struct InfoSyncWriteInst{
    uint16_t addr;
    uint16_t addr_length;
    XELInfoSyncWrite_t* p_xels;
    uint8_t xel_count;
    bool is_info_changed;
    InfoSyncBulkBuffer_t packet;
  } __attribute__((packed)) InfoSyncWriteInst_t;
*/

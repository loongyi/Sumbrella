#include <DynamixelShield.h>

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DEBUG_SERIAL soft_serial
#elif defined(ARDUINO_SAM_DUE) || defined(ARDUINO_SAM_ZERO)
  #define DEBUG_SERIAL SerialUSB
#elif defined(ARDUINO_OpenRB)  // When using OpenRB-150
  //OpenRB does not require the DIR control pin.
  #define DXL_SERIAL Serial1
  #define DEBUG_SERIAL Serial    
#else
  #define DEBUG_SERIAL Serial
#endif

const uint8_t BROADCAST_ID = 254;
const float DYNAMIXEL_PROTOCOL_VERSION = 2.0;
const uint8_t DXL_ID_CNT = 2;
const uint8_t DXL_ID_LIST[DXL_ID_CNT] = {1, 2};
const uint16_t user_pkt_buf_cap = 128;
uint8_t user_pkt_buf[user_pkt_buf_cap];

// Starting address of the Data to read; Present Position = 132
const uint16_t SR_START_ADDR = 132;
// Length of the Data to read; Length of Position data of X series is 4 byte
const uint16_t SR_ADDR_LEN = 4;
// Starting address of the Data to write; Goal Position = 116
const uint16_t SW_START_ADDR = 116;
// Length of the Data to write; Length of Position data of X series is 4 byte
const uint16_t SW_ADDR_LEN = 4;
typedef struct sr_data{
  int32_t present_position;
} __attribute__((packed)) sr_data_t;
typedef struct sw_data{
  int32_t goal_position;
} __attribute__((packed)) sw_data_t;


sr_data_t sr_data[DXL_ID_CNT];
DYNAMIXEL::InfoSyncReadInst_t sr_infos;
DYNAMIXEL::XELInfoSyncRead_t info_xels_sr[DXL_ID_CNT];

sw_data_t sw_data[DXL_ID_CNT];
DYNAMIXEL::InfoSyncWriteInst_t sw_infos;
DYNAMIXEL::XELInfoSyncWrite_t info_xels_sw[DXL_ID_CNT];

//testbench wave:
//netural pos = [2450, -950]
//up pos = [1850, -500]
//down pos = [3050, -1500]

int32_t goal_position[2] = {2450, 3150};
int32_t up_position[2] = {1850, 3650};
int32_t down_position[2] = {3050, 2450};
uint8_t goal_position_index = 0;

DynamixelShield dxl;

//This namespace is required to use DYNAMIXEL Control table item name definitions
using namespace ControlTableItem;

void setup() {
  // put your setup code here, to run once:
 // For Uno, Nano, Mini, and Mega, use UART port of DYNAMIXEL Shield to debug.
  uint8_t i;
  pinMode(LED_BUILTIN, OUTPUT);
  DEBUG_SERIAL.begin(115200);

  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(57600);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DYNAMIXEL_PROTOCOL_VERSION);

  // Turn off torque when configuring items in EEPROM area
  for(i = 0; i < DXL_ID_CNT; i++) {
    dxl.torqueOff(DXL_ID_LIST[i]);
    dxl.setOperatingMode(DXL_ID_LIST[i], OP_POSITION);
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
  for(i = 0; i < DXL_ID_CNT; i++) {
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

  for(i = 0; i < DXL_ID_CNT; i++) {
    info_xels_sw[i].id = DXL_ID_LIST[i];
    info_xels_sw[i].p_data = (uint8_t*)&sw_data[i].goal_position;
    sw_infos.xel_count++;
  }
  sw_infos.is_info_changed = true;
}


void loop() {
  // put your main code here, to run repeatedly:
  
    while (!Serial.available()){
        // when no person,its in here, nothing happens
        //arduino receives serial message when pi +camera detects person
     }
     
     // runs everything below when person detected:

  int x = Serial.readString().toInt();
  /*
   if (x == 1){
    dxl.setGoalPosition(DXL_ID_LIST[0], up_position[0]);}
   else{
   dxl.setGoalPosition(DXL_ID_LIST[1], up_position[1]);}
   delay(1000);
   Serial.print("motor moved");
   dxl.setGoalPosition(DXL_ID_LIST[0], goal_position[0]);
   dxl.setGoalPosition(DXL_ID_LIST[1], goal_position[1]);
   delay(1000);

  uint8_t i,c=0;
  for(i = 0; i < DXL_ID_CNT; i++) {
    sw_data[i].goal_position = up_position[i];
  }
  //sw_data[0].goal_position = goal_position[0];
  //sw_data[1].goal_position = goal_position[1];
  sw_infos.is_info_changed = true;

  c=dxl.syncWrite(&sw_infos);
  //DEBUG_SERIAL.println(c);c=0;
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  delay(500);

  for(i = 0; i < DXL_ID_CNT; i++) {
    sw_data[i].goal_position = goal_position[i];
  }
  //sw_data[0].goal_position = up_position[0];
  //sw_data[1].goal_position = up_position[1];
  sw_infos.is_info_changed = true;

  c=dxl.syncWrite(&sw_infos);
  //DEBUG_SERIAL.println(c);c=0;
  */
  /*
  sw_data[0].goal_position = goal_position[0];
  sw_data[1].goal_position = goal_position[1];
  sw_infos.is_info_changed = true;

  dxl.syncWrite(&sw_infos);
  delay(10);
    DEBUG_SERIAL.print("Present Position(degree) : ");
  DEBUG_SERIAL.println(dxl.getPresentPosition(DXL_ID_LIST[0]));
  DEBUG_SERIAL.println(dxl.getPresentPosition(DXL_ID_LIST[1])); 
  delay(10);*/
  int i= 0;
  float FREQUENCY = 3/PI;
  for (i=0;i<300;i++){
    wave(FREQUENCY);
  }
  //delay(10);

  //sw_data[0].goal_position = goal_position[0];
  //sw_data[1].goal_position = goal_position[1];
  //sw_infos.is_info_changed = true;

  //dxl.syncWrite(&sw_infos);
  //digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  delay(500); 
  Serial.flush();
}

// Sinusoidal wave parameters
#define AMPLITUDE 128     // Amplitude of the sine wave (0-1023)
//#define FREQUENCY 2.0     // Frequency of the sine wave (Hz)
#define PHASE_SHIFT1 0.0   // Phase shift of the sine wave (radians)
#define PHASE_SHIFT2 0.0   // Phase shift of the sine wave (radians)
#define TIME_STEP 0.01    // Time step for each iteration (seconds)

// Function to calculate the sinusoidal angle
float calculateSinusoidalAngle(float amplitude, float frequency, float phaseShift, float time) {
  return amplitude * sin(2 * PI * frequency * time + phaseShift);
}

int wave (float FREQUENCY){
  static float time = 0.0;
// int nn =1;
//  while(nn<100){

  float motor1Angle = calculateSinusoidalAngle(AMPLITUDE, FREQUENCY, PHASE_SHIFT1, time);
  float motor2Angle = calculateSinusoidalAngle(AMPLITUDE, FREQUENCY, PHASE_SHIFT2, time+(PI/2)); // Phase shift by pi/2 for motor 2
  //DEBUG_SERIAL.print(motor1Angle);
  //DEBUG_SERIAL.print("++++");
  //DEBUG_SERIAL.println(motor2Angle);
  // Convert angles to Dynamixel units (0-1023)
  //netural pos = [2450, 3120]
//up pos = [1850, 2520]
//down pos = [3050, 3720]

  //int motor1Goal = map((int)motor1Angle, -AMPLITUDE, AMPLITUDE, 1850, 3050);
  //int motor2Goal = map((int)motor2Angle, -AMPLITUDE, AMPLITUDE, 2520, 3720);

  int motor1Goal = map((int)motor1Angle, -AMPLITUDE, AMPLITUDE, 0, 4095);
  int motor2Goal = map((int)motor2Angle, -AMPLITUDE, AMPLITUDE, 2520, 3720);
 // DEBUG_SERIAL.print(motor1Goal);
 // DEBUG_SERIAL.print("[][][]");
 // DEBUG_SERIAL.println(motor2Goal);

  sw_data[0].goal_position = motor1Goal;
  sw_data[1].goal_position = motor2Goal;
  sw_infos.is_info_changed = true;

   // Increment time for the next iteration
  time += TIME_STEP;
  //nn+=1;
  dxl.syncWrite(&sw_infos);
  delay(10); // Add a small delay between iterations
  //}
}

/* syncRead
  Structures containing the necessary information to process the 'syncRead' packet.

  typedef struct XELInfoSyncRead{
    uint8_t *p_recv_buf;
    uint8_t id;
    uint8_t error;
  } __attribute__((packed)) XELInfoSyncRead_t;

  typedef struct InfoSyncReadInst{
    uint16_t addr;
    uint16_t addr_length;
    XELInfoSyncRead_t* p_xels;
    uint8_t xel_count;
    bool is_info_changed;
    InfoSyncBulkBuffer_t packet;
  } __attribute__((packed)) InfoSyncReadInst_t;
*/

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
#include <Arduino.h>
#define VERSION "FocusBuddy TopBox V3.231203 by keithrickard@hotmail.com"  // Previous version V3.231018
/* For the Arduino Nano

  Nano Connections
 B ================
  
                  +-------+
    RB_STAT   D13 o       o D12   btRXD
              3V3 o       o D11~  btTXD
              REF o       o D10~  LED
Be  RELAY1    A0  o       o D9~   LED_GN (PWR_ON)
Pe  RELAY2    A1  o       o D8    BTN_GND Gy
Gy  RELAY3    A2  o       o D7    BTN_OUT Pe
W   RELAY4    A3  o       o D6~   H1_A1A    (HA_A1B connected to GND)
Bk  MPU_SDA   A4  o       o D5~   H2_A1A    (HB_A2B connected to GND)
W   MPU_SCL   A5  o       o D4    BTN_IN  Be
    RB_BTNS   A6  o       o D3~   F_B1B   O
R   PWR_ON    A7  o       o D2    F_B1A   R
Gn  RELAY5V   5V  o  ICSP o GND           Bn
              RST o  ooo  o RST
Bk  RELAYGND  GND o 1ooo  o RX0
              VIN o       o TX1
                  +-------+

      ICSP
      ooo      5V  D11  GND     R  Y  O
     1ooo      D12 D13  RST     Gn Be
            
R      bt5V     5V
Gy     btTXD    D11
Bn     btRXD    D12
Bk     btGND    GND
Gn     RB_STAT   D13
*/

#define RB_STAT       13
#define btRXD         12
#define btTXD         11
#define LED_GND       10
#define LED           9
#define BTN_GND       8
#define BTN_IN        7         // Button to move focuser IN
#define HA_A1A        6         // Dew Heater A PWM
#define HB_A1A        5         // Dew Heater B PWM
#define BTN_OUT       4         // Button to move focuser OUT
#define F_A1B         3         // HIGH: tip is -ve
#define F_A1A         2         // HIGH: tip is +ve
#define RELAY4        A3        // Focuser motor relay
#define RELAY3        A2        // Stepper motor power
#define RELAY2        A1        // EL Panel power %
#define RELAY1        A0        // PS_ON
#define BTNS          A6        // Buttons to open/close the roof manually
#define PWR_ON        A7        // Senses +5V line from the PSU

#define BLUETOOTH     0
#define SERIAL_PORT   1
#define EE_BACKLASH   4
#define EE_DEW_A      6
#define EE_DEW_B      7
#define EE_FOCPOSPTR  8
#define EE_PRESETS    10
#define WATCHPERIOD   3000
#define RB_PERIOD     1000
#define TOLERANCE     48        // Accelerometer tolerance for telescope safety
#define LED_DIM       4
#define LED_ON        255
#define BTN_OPEN      970       // Analogue value for OPEN button
#define BTN_CLOSE     50        // Analogue value for CLOSE button

#define OUT           0
#define IN            1

#define STOP          0
#define GO            1
#define HALT          2
#define DONE          3
#define GOTO          4
#define HOME          5
#define SLOT          6

#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <Wire.h>
#include <MPU6050_light.h>
SoftwareSerial RB(btTXD, btRXD);    // RoofBuddy
MPU6050 mpu0(Wire);
MPU6050 mpu1(Wire);

const long MAX_LIMIT = 1000000;
byte currDir   = STOP;
byte currSpeed = STOP;
byte RBcomms = 0;
char cmd;
byte dir, revPol;
byte pcWatch = 0;
char rbBtn = 'h';

byte heaters[2] = {0, 0};
const byte relayPins[4] = {RELAY1, RELAY2, RELAY3, RELAY4};
byte state[4] = {INPUT, INPUT, INPUT, INPUT}; // Relays status
bool isMoving, btn;
unsigned long focTimer, focStart, tempPos;
unsigned long pcWatchTimer = 0;
unsigned long updateTimer = 0;
unsigned long timerStart = 0;           // Records millis() when power timer started
unsigned long rbTimer = 0;              // Sends telescope safety info to RoofBuddy
long timerPeriod = 0;                   // 0 = power timer off, > 0 = timer period in milliseconds
long focPos, focTarg, maxLimit;
int backlash;

struct ee{
  long preset[10];
  char name[10][32];
} ee;

struct parked{
  float x0off = 0;
  float y0off = 0;
  float x1off = 0;
  float y1off = 0;
} parked;

void setup() {
  pinMode(RB_STAT,  INPUT);
  pinMode(LED,      OUTPUT);
  pinMode(LED_GND,  OUTPUT);
//pinMode(PWR_ON,   INPUT);
  pinMode(F_A1A,    OUTPUT);
  pinMode(F_A1B,    OUTPUT);
  pinMode(HA_A1A,   OUTPUT);
  pinMode(HB_A1A,   OUTPUT);
  pinMode(BTN_GND,  OUTPUT);
  pinMode(BTN_IN,   INPUT_PULLUP);
  pinMode(BTN_OUT,  INPUT_PULLUP);
  for (byte i = 0; i <= 3; i++){
    pinMode(relayPins[i], INPUT);
  }
  digitalWrite(LED_GND, LOW);
  analogWrite(LED,      LED_DIM);
  digitalWrite(F_A1A,   LOW);
  digitalWrite(F_A1B,   LOW);
  digitalWrite(BTN_GND, LOW);

  RB.begin(9600);
  RB.setTimeout(500);
  Serial.begin(9600);
  Serial.setTimeout(100);
  Wire.begin();
  mpu0.setAddress(0x68);
  mpu1.setAddress(0x69);
  mpu0.setAccConfig(0); 
  mpu1.setAccConfig(0);
  mpu0.begin();
  mpu1.begin();
  getMPUpos();            // Get MPU telescope parked postion offsets

  isMoving = STOP;
  tempPos = 0;
  btn = 0;
  eepromSetUp();
}
// ==============================================================================================
void loop() {
  commands();                                 // See if a command has been received via the Serial port

  if (isMoving == GO) {
    if (!pwrOn()) {                           // Stop immediately if power is lost
      focuser(HALT);
    }
    else {
      if (millis() >= updateTimer) {          // Save focPos every 1000ms to EEPROM in case of failure
        updateTimer = millis() + 1000;
        long tempFocPos = focPos;             // Save original focPos
        pcWatch *= 2;
        focPos = focuserGetPos();             // Get new postion
        pcWatch /= 2;
        eepromFocPos();                       // Save this to EEPROM
        focPos = tempFocPos;                  // Restore focPos
      }
      if (focTimer) if (millis() >= focTimer) focuser(DONE);  // Service any focuser action
    }
  }
  if (pcWatch) if (millis() > pcWatchTimer) focuser(HALT);  // Stop focuser if PC is not watching
  
  byte btn_in = !digitalRead(BTN_IN);         // Act on any buttons being pressed
  byte btn_out = !digitalRead(BTN_OUT);
  if (btn_in || btn_out) {
    if (pwrOn()){
      analogWrite(LED, LED_ON);
      if (!btn) {                             // Only start the motor if not already moving
        focuser(btn_in ? IN : OUT);           // Move the focuser depending on button pressed
        btn = 1;                              // Motor now moving as button is being pressed
      }
    }
  }
  else {
    if(btn) {                               // Neither button is being pressed - motor moving still?
      if(isMoving == GO) focuser(HALT);     // Stop focuser if moving
      btn = 0;
    }
  }

  if (timerPeriod) {                                        // If timerPeriod > 0 then power timer is active
    if(timerPeriod - long(millis() - timerStart) <= 0) {    // Has timerPeriod expired? (timerStart eliminates fear of rollover)
      pinMode(RELAY1, state[0] = OUTPUT);                   // If so, turn off the PSU power
      timerPeriod = 0;                                      // timerPeriod = 0 to switch off timer
    }
  }  

  roofBuddy();                                              // Service RoofBuddy
  serviceHeaters();                                         // Ensure dew heaters are receiving correct power
}
// ==============================================================================================
void commands() {
  cmd = 0;
  if(Serial.available()) cmd = Serial.read();       // See if a command has been received from the PC
  if (!cmd) return;
  
  switch (cmd) {                                    // Process the command
    case '\n': cmd = 0;                     break;  // Soak up this character
    case '\r': cmd = 0;                     break;  // Soak up this character
    case 'A':                               break;  // Simply echos 'A'
    case '$': switches();                   break;  // Switch relays
    case '#': heaterControls();             break;  // Dew heater controls
    case 'V': version();                    break;
    case 'v': version();                    break;
    case 'O': pcWatch = 1; focuser(OUT);    break;  // Send focuser outwards (distance may be given)
    case 'o': focuser(OUT);                 break;
    case 'I': pcWatch = 1; focuser(IN);     break;  // Send focuser inwards (distance may be given)
    case 'i': focuser(IN);                  break;
    case 'Q': focuser(HALT);                break;  // Stop focuser immediately
    case 'q': focuser(HALT);                break;
    case 'G': pcWatch = 1; focuser(GOTO);   break;  // Go fast to given position
    case 'g': focuser(GOTO);                break;
    case 'H': pcWatch = 1; focuser(HOME);   break;  // Go fast to home (preset 0)
    case 'h': focuser(HOME);                break;
    case 'S': pcWatch = 1; focuser(SLOT);   break;  // Goto a stored focuser position in preset 0 - 9
    case 's': focuser(SLOT);                break;
    case 'P': focuserGetPos();              break;  // Get focuser's position
    case 'p': focuserSetPos();              break;  // Set focuser's position
    case 'L': cmd = dir ? 'I' : 'O';        break;  // Get last direction
    case 'l': cmd = dir ? 'I' : 'O';        break;
    case 'r': clockGet();                   break;  // Get Arduino's millis() clock
    case 'R': reset();                      break;  // Soft reset arduino and its millis() timer
    case 'X': timerSet();                   break;  // Stop or start timer for so many milliseconds for power off.
    case 'x': timerGet();                   break;  // Get timer remaining on timer.
    case 'T': targetGet();                  break;  // Get focuser target
    case 't': targetGet();                  break;
    case 'B': backlashGet();                break;  // Get backlash allowance: -ve Inwards, +ve Outwards, 0 None
    case 'b': backlashSet();                break;  // Set backlash allowance: -ve Inwards, +ve Outwards, 0 None
    case 'N': eepromGetName();              break;  // Get name for given preset
    case 'n': eepromSetName();              break;  // Set name for given preset
    case 'E': eepromGetPreset();            break;  // Get stored focuser position in preset 0 - 9
    case 'e': eepromSetPreset();            break;  // Store current focuser position in preset 0 - 9
    case 'D': eepromGetPresets();           break;  // Get all preset values
    case 'd': eepromGetPresets();           break;
    case 'Z': eepromTempPos();              break;  // Temporary store for setting a EERPROM preset
    case 'z': eepromTempPos();              break;
    case 'W': eepromWipe();                 break;  // Wipe EEPROM storage
    case 'w': eepromWipe();                 break;
    case 'F': eepromSetUp();                break;  // Format EPROM storage structure
    case 'f': eepromSetUp();                break;
    case 'C': eepromDisp();                 break;  // Send complete contents of the EEPROM
    case 'c': eepromDisp();                 break;
    case 'M': cmd = (isMoving == GO) + '0'; break;  // Returns 1 if focuser is moving, 0 is not
    case 'm': cmd = (isMoving == GO) + '0'; break;
    case '>': revPol = 0; eepromFocPos();   break;  // +ve motor connector tip for outward moves
    case '<': revPol = 1; eepromFocPos();   break;  // -ve motor connector tip for outward moves
    case '|': cmd = (revPol ? '-' : '+');   break;  // Get motor tip polarity for outward moves
    case '@': mpu6050();                    break;  // MPU6050 commands
    case '^': roofBuddyCmds();              break;  // RoofBuddy commands
    case '?': help();                       break;
    case '~': cmd = (pwrOn()) + '0';        break;  // 1 = PSU power is on, 0 = PSU power is off.
    default: cmd = 0;                       break;  // Uh? Command is not valid so no action
  }
  if (cmd) Serial.println(cmd);                     // Echo the command or '%' if there is an error
}
// ==============================================================================================
void focuser(byte action) {
  if (btn == 1 && action != DONE && action != HALT) return; // Quit if button move and action is not to stop
  digitalWrite(F_A1A, LOW);                                 // Cut power to the motor
  digitalWrite(F_A1B, LOW);
  pinMode(RELAY4, INPUT);                                   // Brake the motor
  analogWrite(LED, LED_DIM);
  focTimer = 0;                                             // Reset focuser timer
  
// UPDATING FOCUSER POSITION
  if (isMoving == GO) {
    long newFocPos = focPos + (millis() - focStart) * ((focTarg > focPos) ? 1 : -1);
    focPos = (newFocPos < 1 ? 1 : newFocPos);
  }
  isMoving = STOP;
  if (action == DONE) focPos = focTarg;                 // Planned Stop
  if (focPos > maxLimit) focPos = maxLimit;
  eepromFocPos();                                       // Update EEPROM with current focuser postion
  if (action == HALT || action == DONE) {
    pcWatch = 0;
    return;                                             // Quit if aborted move or planned stop
  }
  
  if (!pwrOn()) {
    cmd = '%';
    return;                 // Quit if there is no power - unable to move the focuser!
  }

// DETERMINE HOW TO MOVE TO THE FOCUSER
  if (action == SLOT || action == HOME) {                                  // See if need to go to a slot preset
    int e = (action == SLOT ? Serial.parseInt(): 0);                       // Get slot number or use Home position
    if (e >= 0 && e <= 9){
      EEPROM.get(EE_PRESETS, ee);
      focTarg = (ee.preset[e]);
    } else {
      cmd = '%';
      return;
    }
  } 
  else {                                                                    // Slot not selected - act accordingly
    focTarg = Serial.parseInt();                                            // See if a value has been provided (0 if not)
    if (action == IN || action == OUT) {                                    // Act if relative move
      if (focTarg) focTarg = focPos + (action == IN ? -focTarg : focTarg);  // Convert target to absolute position if not 0
      else focTarg = ((action == IN) ? 1 : maxLimit);                       // Else select lower or upper limit
    }
    if (focTarg < 1) focTarg = 1;                                           // Make sure target is within limits
    if (focTarg > maxLimit) focTarg = maxLimit;
    if (focTarg == focPos) return;                                          // Quit if target is same as current postion
  }
  
  dir = focTarg < focPos;                                                   // Figure out which direction to send the motor
  focTimer = abs(focPos - focTarg) + (focStart = millis());                 // Figure out time of travel

// GET FOCUSER MOVING
  analogWrite(LED, LED_ON);
  int a = F_A1A;
  int b = F_A1B;
  if (revPol) {
    a = F_A1B;
    b = F_A1A;
  }

  pinMode(RELAY4, OUTPUT);                                                  // Open connection to motor's terminals
  digitalWrite((dir ? b : a), HIGH);                                        // dir: 1 = Outwards, 0 = Inwards
  isMoving = GO;                                                            // Focuser is moving
  if (pcWatch == 1) pcWatchTimer = millis() + WATCHPERIOD;
}
// ==============================================================================================
unsigned long focuserGetPos(){
  cmd = 0;                                                                  // Don't echo command
  unsigned long position = focPos;
  if (isMoving == GO) position +=  (millis() - focStart) * (focTarg < focPos ? -1 : 1);
  Serial.println(position);
  if (pcWatch == 1) pcWatchTimer = millis() + WATCHPERIOD;
  return position;
}
// ==============================================================================================
void focuserSetPos() {
  long pos = Serial.parseInt();
  if (pos > maxLimit) pos = maxLimit;
  if (pos > 0) {
    focPos = pos;
    eepromFocPos();
  }
}
// ==============================================================================================
void targetGet() {
  cmd = 0;
  Serial.println(focTarg);
}
// ==============================================================================================
void backlashGet() {
  cmd = 0;
  EEPROM.get(EE_BACKLASH, backlash);
  Serial.println(backlash);
}
// ==============================================================================================
void backlashSet() {
  int b = Serial.parseInt();
  if (b >= -10000 && b <= 10000) backlash = b;
  EEPROM.put(EE_BACKLASH, backlash);
}
// ==============================================================================================
void clockGet(){
  cmd = 0;
  Serial.println(millis());
}
// ==============================================================================================
void timerSet(){  // Xn command
  timerPeriod = Serial.parseInt();
  timerStart = millis();
}
// ==============================================================================================
void timerGet(){  // x command
  cmd = 0;
  long timerLeft = timerPeriod - long(millis() - timerStart);
  if(timerLeft < 0) {
    timerPeriod = timerLeft = 0;
    timerStart = millis();
  }
  Serial.println(timerLeft);
}
// ==============================================================================================
void eepromTempPos() {
  long pos = Serial.parseInt();
  if (pos > 0) tempPos = pos;
}
// ==============================================================================================
void eepromGetPreset(){
  char buf[2]; 
  Serial.readBytes(buf, 1);
  byte e = buf[0] - '0';
  if (e <= 9) {
    cmd = 0;
    EEPROM.get(EE_PRESETS, ee);
    long pos = (ee.preset[e]);
    Serial.println(pos);      
  }
  else cmd = '%';
}
// ==============================================================================================
void eepromSetPreset() {
  char buf[2]; 
  Serial.readBytes(buf, 1);
  byte e = buf[0] - '0';
  if (isMoving == STOP) {
    if (e <= 9) {
      if (!tempPos) tempPos = focPos;
      Serial.println(tempPos);
      EEPROM.get(EE_PRESETS, ee);
      ee.preset[e] = tempPos;
      EEPROM.put(EE_PRESETS, ee);
      tempPos = 0;
    }
  }
  else cmd = '%';
}
// ==============================================================================================
void eepromGetName() {
  char buf[2]; 
  Serial.readBytes(buf, 1);
  byte e = buf[0] - '0';
  if (e <= 9) {
    EEPROM.get(EE_PRESETS, ee);
    cmd = 0;
    Serial.println(ee.name[e]);
  }
  else cmd = '%';
}
// ==============================================================================================
void eepromSetName() {
  char buf[2]; 
  Serial.readBytes(buf, 1);
  byte e = char(buf[0]) - '0';
  if (e >= 1 && e <= 8) {                                     // 0 and 9 are not adjustable (Home and Upper Limit)
    String name = Serial.readStringUntil('#');
    EEPROM.get(EE_PRESETS, ee);
    for (byte i = 0; i <= 30; i++) {                          // Only accept 31 chars for the preset name
      ee.name[e][i] = '\0';
      ee.name[e][i + 1] = '\0';
      if (i == name.length() || name.charAt(i) == '\r' || name.charAt(i) == '\n') break;
      ee.name[e][i] = name.charAt(i);
    }
    EEPROM.put(EE_PRESETS, ee);
  }
  else {
    cmd = '%';
    while (Serial.available()) Serial.read();                 // Clear the applicable serial buffer
  }
}
// ==============================================================================================
void eepromSetUp() {
  unsigned long check;
  unsigned int ptr;
  EEPROM.get(EE_FOCPOSPTR, ptr);      // Load what is already in the EEPROM
  revPol = bitRead(ptr, 15);          // Get polarity info for driving the motor
  bitWrite(focPos, 15, 0);            // Clear the bit
  EEPROM.get(ptr, focPos);
  EEPROM.get(EE_BACKLASH, backlash);
  EEPROM.get(EE_DEW_A, heaters[0]);
  EEPROM.get(EE_DEW_B, heaters[1]);
  EEPROM.get(EE_PRESETS, ee);
  dir = bitRead(focPos,23);           // Bit 23 is last direction: 1 = inwards, 0 = outwards
  bitWrite(focPos, 23, 0);            // Clear bit 23 which holds last direction
  maxLimit = ee.preset[9];

  EEPROM.get(0, check);               // See if EEPROM has been formatted
  if (check == 0x0D6015EE) return;    // EE(PROM) I5 600D i.e. already set up - so quit
  Serial.print("Formating EEPROM... ");
  delay(1000);
  
  eepromWipe();                                         // Start with a clean slate
  EEPROM.put(0, (unsigned long)0x0D6015EE);             // Format the EERPOM
  focPos = MAX_LIMIT / 2;                               // Set-up the 10 focus position presets
  heaters[0] = heaters[1] = dir = revPol = backlash = 0;
  for (byte i = 0; i <= 8; i++) ee.preset[i] = focPos;  // Initialise the preset slots
  ee.preset[9] = MAX_LIMIT;
  strcpy(ee.name[0], "Home");
  strcpy(ee.name[1], "Preset 1");
  strcpy(ee.name[2], "Preset 2");
  strcpy(ee.name[3], "Preset 3");
  strcpy(ee.name[4], "Preset 4");
  strcpy(ee.name[5], "Preset 5");
  strcpy(ee.name[6], "Preset 6");
  strcpy(ee.name[7], "Preset 7");
  strcpy(ee.name[8], "Preset 8");
  strcpy(ee.name[9], "Upper Limit");
  EEPROM.put(EE_PRESETS, ee);                                 // Save presets to the EEPROM
  EEPROM.put(EE_BACKLASH, backlash);                          // Save backlash allowance
  EEPROM.put(EE_DEW_A, heaters[0]);                           // Save dew heater A power value
  EEPROM.put(EE_DEW_B, heaters[1]);                           // Save dew heater B power value
  EEPROM.put(EE_FOCPOSPTR, (int)(sizeof(ee) + EE_PRESETS));   // Save pointer for the current focuser position
  eepromFocPos();                                             // Save the focuser position
  Serial.println("done");
}
// ==============================================================================================
void eepromWipe() {
  for (int i = 0; i <= 1023; i++) EEPROM.put(i, (byte)0xFF);
}
// ==============================================================================================
void eepromDisp() {
  byte b;
  Serial.println();
  for (int i = 0; i <= 1023; i += 8) {
    hexDigits(i >> 8);
    hexDigits(i & 0xFF);
    Serial.print(" ");
    for (int j = 0; j <= 7; j++){
      hexDigits(EEPROM.read(i + j));
      Serial.print(" ");
    }
    for (int j = 0; j <= 7; j++) {
      b = EEPROM.read(i + j);
      if (b < 32 || b > 126) b = '.';
      Serial.print(char(b));
    }
    Serial.println();
    cmd = 0;
  }  
}
// ==============================================================================================
void hexDigits(byte b){
  if (b < 16) Serial.print('0');
  Serial.print(b, HEX);
}
// ==============================================================================================
void eepromGetPresets() {
  unsigned long check;

  cmd = '%';                          // Assume format error
  EEPROM.get(0, check);               // See if EEPROM has been formatted
  if (check != 0x0D6015EE) return;    // EE(PROM) I5 600D.  If not this then EEPROM not formatted. Quit
  
    Serial.print("\nBacklash Allowance   : ");
    Serial.println(backlash);
    Serial.print("Last movement        : ");
    Serial.print(dir ? "Out" : "In");
    Serial.print("wards\nmillis()             : ");
    Serial.println(millis());
    Serial.print("Polarity on out moves: ");
    Serial.println(revPol ?  "-ve" : "+ve");
    Serial.print("\nP ");
  focuserGetPos();
  for (byte i = 0; i <=9; i++) {
    Serial.print(i);
    Serial.print(' ');
    Serial.print(ee.preset[i]);
    Serial.print(' ');
    Serial.println(ee.name[i]);
  }
  cmd = 0;
}
// ==============================================================================================
void eepromFocPos() {
  unsigned int ptr;                   // Save Focuser position to EEPROM
  unsigned long n;
  unsigned long eeFocPos = focPos;
  bitWrite(eeFocPos, 23, dir);        // Bit 23 is set to value of last focuser direction (1 = IN, 0 = OUT)
  EEPROM.get(EE_FOCPOSPTR, ptr);      // Get location pointer for storing focus position

  while (ptr <= (1020 - sizeof(parked))){; 
    bitWrite(ptr, 15, revPol);        // Update revPol if necessary
    EEPROM.put(EE_FOCPOSPTR, ptr);    // Update ptr
    bitWrite(ptr, 15, 0);

    EEPROM.get(ptr, n);               // Get last recorded postion
    if (n == eeFocPos) break;         // If it is the same as current then return
    
    EEPROM.put(ptr, eeFocPos);        // Store the focuser position
    EEPROM.get(ptr, n);               // See if it was stored OK
    if (n == eeFocPos) break;         // Quit if it has
    ptr ++;                           // Otherwise move along to the next location and try again until space runs out

  }
}
/*
  The EEPROM can only be re-written to about 100,000 times.  The focus position is written everytime
  a move is completed, or aborted, to the EEPROM to remember the focuser's postion.
  
  If focPos is not stored correctly, then the storage pointer is moved along one. Logically, it should
  be moved along 4 bytes as 4 bytes are required to save focPos, but the EEPROM may still be able to 
  save it from the next byte rather in the next 4 along, so extending the life of the EEPROM.
  
  Theorectically, focPos should be able to be written to EEPROM about 21 million times before it gives up!
*/
// ==============================================================================================
void reset() {
  Serial.println(F("Resetting"));
  asm volatile ("jmp 0");
}
// ==============================================================================================
bool pwrOn() {
  return analogRead(PWR_ON) > 511;
  return digitalRead(PWR_ON);
}
// ==============================================================================================
void switches() {   // $ command
  char buf[3];
  cmd = '%';
  if(Serial.readBytes(buf, 2) < 2) return;
  byte relay = buf[0] - '1';
  if (relay > 3) return;                                                  // Only 1, 2, 3 are valid
  cmd = '$';
  if (buf[1] == '+') pinMode(relayPins[relay], state[relay] = OUTPUT);   // Switch to NO position
  if (buf[1] == '-') pinMode(relayPins[relay], state[relay] = INPUT);    // Switch to NC position
  if (buf[1] == '=') {
    char c = (state[relay]) == INPUT ? '-' : '+';
    Serial.println(c);
    cmd = 0;
  }
}
// ==============================================================================================
void heaterControls() {    // # command (#Annnn or #Bnnnn)
  char buf[2];
  cmd = '%';                                          // Assume there will be an error
  if(Serial.readBytes(buf, 1) < 1) return;
  
  byte heater = buf[0] - 'A';                         // Get heater number
  if (heater > 2) return;                             // Only A or B are valid
  delay(100);
  if (Serial.peek() == '=') {                         // Value is to be got
    EEPROM.get(EE_DEW_A, heaters[0]);
    EEPROM.get(EE_DEW_B, heaters[1]);
    Serial.println(heaters[heater]);
    cmd = 0;
    return;
  }
  int value = (Serial.parseInt());                    // Value is to be set
  if (value > 255) return;
  analogWrite(heater ? HA_A1A : HB_A1A, heaters[heater] = byte(value));
  EEPROM.put(EE_DEW_A, heaters[0]);
  EEPROM.put(EE_DEW_B, heaters[1]);
  cmd = '#';
}
// ==============================================================================================
void serviceHeaters() {                     // Ensure heaters are receiving correct power
  EEPROM.get(EE_DEW_A, heaters[0]);
  analogWrite(HA_A1A, heaters[0]);
  EEPROM.get(EE_DEW_B, heaters[1]);
  analogWrite(HB_A1A, heaters[1]);
}
// ==============================================================================================
void version() {
  cmd = 0;
  Serial.println(F(VERSION));
}
// ==============================================================================================
void help() {
  version();
  Serial.println(F("\nACTION COMMANDS"));
  Serial.println(F("Q q : Stop focuser immediately"));
  Serial.println(F("O   : Move outwards to end of travel *"));
  Serial.println(F("OX  : Move outwards for X steps (1-10000) *"));
  Serial.println(F("o   : Move outwards to end of travel"));
  Serial.println(F("oX  : Move outwards for X steps (1-10000)"));
  Serial.println(F("I   : Move inwards to end of travel *"));
  Serial.println(F("IX  : Move inwards for X steps (1-10000) *"));
  Serial.println(F("i   : Move inwards to end of travel"));
  Serial.println(F("iX  : Move inwards for X steps (1-10000)"));
  Serial.println(F("GX  : Goto position X *"));
  Serial.println(F("gX  : Goto position X"));
  Serial.println(F("H   : Goto to Home postion *"));
  Serial.println(F("h   : Goto to Home postion"));
  Serial.println(F("SX  : Goto preset X *"));
  Serial.println(F("sX  : Goto preset X"));
  Serial.println(F("xN  : N=0 stop PSU timer, N>0 start PSU timer for N ms"));
  Serial.println(F("R   : Restart FocusBuddy and its millis() clock"));
  Serial.println(F("#X= : Get Dew Band X (A or B) power value (0-255)"));
  Serial.println(F("#XN : Set Dew Band X (A or B) power value N (0-255"));
  Serial.println(F("$X= : Get Relay X (1-4) status: '+' = closed, '-' = open"));
  Serial.println(F("$X- : Set Relay X (1-4) status to closed"));
  Serial.println(F("$X+ : Set Relay X (1-4) status to open"));
  Serial.println(F("~   : Senses the main power supply. 1 = on, 0 = off"));
  Serial.println(F("\n* Note: focuser stops after 3 secs if no comms from PC"));
  Serial.println(F("\nPROPERTIES"));
  Serial.println(F("P   : Get focuser position"));
  Serial.println(F("pX  : Set focuser position"));
  Serial.println(F("T t : Get target position"));
  Serial.println(F("L l : Get last move direction"));
  Serial.println(F("M m : Is focuser moving? 1 = yes, 0 = no"));
  Serial.println(F(">   : Default polarity for focuser motor (+)"));
  Serial.println(F("<   : Reverse polarity for focuser motor (-)"));
  Serial.println(F("|   : Get polarity: + or - "));
  Serial.println(F("bX  : Set backlash allowance: -ve Inwards, +ve Outwards, 0 None"));
  Serial.println(F("B   : Get backlash allowance"));
  Serial.println(F("X   : Get timer left in ms of power timer"));
  Serial.println(F("r   : Get Arduino's millis() clock"));
  Serial.println(F("V v : Get FocusBuddy Verion"));
  Serial.println(F("?   : Get this help page"));
  Serial.println(F("\nEEPROM COMMANDS"));
  Serial.println(F("nXY#: Set preset X (1-8) name to Y, terminated by #"));
  Serial.println(F("NX  : Get preset X (0-9) name"));
  Serial.println(F("Z z : Provide focuser postion for storing"));
  Serial.println(F("eX  : Set preset X (1-8) position to that provided by Z command"));
  Serial.println(F("Ex  : Get preset X (0-9) position"));
  Serial.println(F("D d : Get all presets info and other settings"));
  Serial.println(F("W w : Wipe EEPROM storage"));
  Serial.println(F("F f : Format EEPROM storage structure"));
  Serial.println(F("C c : Get complete contents of EEPROM in HEX"));
  Serial.println(F("\nACCELEROMETER COMMANDS"));
  Serial.println(F("@0X @1X @X : Get Acc X value from MPU0, MPU1 or both"));
  Serial.println(F("@0Y @1Y @Y : Get Acc Y value from MPU0, MPU1 or both"));
  Serial.println(F("@0Z @1Z @Z : Get Acc Z value from MPU0, MPU1 or both"));
  Serial.println(F("@0x @1x @x : Get Gyro x value from MPU0, MPU1 or both"));
  Serial.println(F("@0y @1y @y : Get Gyro y value from MPU0, MPU1 or both"));
  Serial.println(F("@0T @1T @T : Get Temperature from MPU0, MPU1 or both"));
  Serial.println(F("@I:          Intilialise MPUs and set offsets"));
  Serial.println(F("@S:          Save MPU X & Y offsets to EEPROM"));
  Serial.println(F("@G:          Get MPU X & Y offsets from EEPROM"));
  Serial.println(F("@*:          Returns 1 if MPUs X & Y are at park postion, 0 if not"));
  Serial.println(F("\nROOF BUDDY"));
  Serial.println(F("Command format ^asc[=x]$"));
  Serial.println(F("asc is the command code in ASCII, e.g. e is 101"));
  Serial.println(F("[=x] is for optionally passing on a numerical value"));
  Serial.println(F("$ is the command termninator - all responses end with this too"));
  Serial.println(F("^O$   : Open roof"));
  Serial.println(F("^o$   : Open roof while TopBox button is pressed"));
  Serial.println(F("^C$   : Close roof"));
  Serial.println(F("^c$   : Close roof while TopBox button is pressed"));
  Serial.println(F("^S$   : Scope is in safe position per MPUs = 1, not = 0"));
  Serial.println(F("^s$   : Roof status"));
  Serial.println(F("^w$   : Switch status: 'O' open, 'C' closed, 'M' neither"));
  Serial.println(F("^b$   : Battery voltage"));
  Serial.println(F("^R$   : Reset rain sensor"));
  Serial.println(F("^r$   : Rain sensor wet = 1, dry = 0"));
  Serial.println(F("^H$   : Stop roof moving immediatley"));
  Serial.println(F("^h$   : Stop roof moving slowly"));
  Serial.println(F("^l$   : Scope connected to RoofBuddy? 1 = yes, 0 = no"));
  Serial.println(F("^P$   : Park telesope"));
  Serial.println(F("^p$   : Is scope parked? 1 = yes, 0 = no"));
  Serial.println(F("^V$   : RoofBuddy version"));
  Serial.println(F("^:x#  : Send command to LX200 and get response"));
  Serial.println(F("^d$   : Get scope's parked Declination"));
  Serial.println(F("^h$   : Get scope's parked Hour Angle"));
  Serial.println(F("^D=x$ : Set scope's parked Declination x"));
  Serial.println(F("^H=x$ : Set scope's parked Hour Angle x"));
  Serial.println(F("\n In all cases '%' is returned for invalid commands"));
}
// ==============================================================================================
void mpu6050() {    // Prefix '@'
  mpu0.update();
  mpu1.update();
  delay(25);
  if(Serial.available()) cmd = Serial.read();                   // Get command from PC
  if (!cmd) return;                                             // Return if nothing received
  switch (cmd) {
    case 'e':                   break;    // @e Echo check
    case 'I': mpuInitialise();  break;    // @I Initialise MPUs (offsets = current MPU values)
    case 'R': resetOffsets();   break;    // Set offsets to 0
    case 'o': getOffsets();     break;    // Return offsets
    case 'g': getReadings();    break;
    case '0': readMPU(cmd);     break;    // @0 Prefix for MPU 0
    case '1': readMPU(cmd);     break;    // @1 Prefix for MPU 1
    case 'X': readMPU(cmd);     break;    // @X Get AccX / pitch
    case 'Y': readMPU(cmd);     break;    // @Y Get AccY / roll
    case 'Z': readMPU(cmd);     break;    // @Z Get AccZ / yaw
    case 'x': readMPU(cmd);     break;    // @x Get GyroX
    case 'y': readMPU(cmd);     break;    // @y Get GyroY
    case 'z': readMPU(cmd);     break;    // @z Get GyroZ
    case 'T': readMPU(cmd);     break;    // @T Get Temperature (Celsius)
    case '*': scopeSafety();    break;    // @* '1' = telescope is in its parked position and safe, '0' = not
    case 'S': saveMPUpos();     break;    // @S Save current MPU-6050's X & Y axes values to EERPOM
    case 'G': getMPUpos();      break;    // @G Get MPU-6050's starting X & Y axes values to EERPOM
    default: cmd = 0;           break;
  }
}
// ----------------------------------------------------------------------------------------------
void mpuInitialise() {
  mpu0.calcAccOffsets();
  parked.x0off = mpu0.getAccXoffset();
  parked.y0off = mpu0.getAccYoffset();
  mpu1.calcAccOffsets();
  parked.x1off = mpu1.getAccXoffset();
  parked.y1off = mpu1.getAccYoffset();
}
// ----------------------------------------------------------------------------------------------
void resetOffsets() {
  mpu0.setAccOffsets(parked.x0off = 0, parked.y0off = 0, 0);
  mpu1.setAccOffsets(parked.x1off = 0, parked.y1off = 0, 0);
}
// ----------------------------------------------------------------------------------------------
void getOffsets() {
  Serial.println(F("OFFSETS"));
  Serial.print(F("MPU 0 X: "));
  Serial.println(mpu0.getAccXoffset());
  Serial.print(F("MPU 0 Y: "));
  Serial.println(mpu0.getAccYoffset());
  Serial.print(F("MPU 0 Z: "));
  Serial.println(mpu0.getAccZoffset());
  Serial.print(F("MPU 1 X: "));
  Serial.println(mpu1.getAccXoffset());
  Serial.print(F("MPU 1 Y: "));
  Serial.println(mpu1.getAccYoffset());
  Serial.print(F("MPU 1 Z: "));
  Serial.println(mpu1.getAccZoffset());
}
// ----------------------------------------------------------------------------------------------
void getReadings() {
  Serial.println(F("READINGS"));
  Serial.print(F("MPU 0 X: "));
  Serial.println(mpu0.getAccX());
  Serial.print(F("MPU 0 Y: "));
  Serial.println(mpu0.getAccY());
  Serial.print(F("MPU 0 Z: "));
  Serial.println(mpu0.getAccZ());
  Serial.print(F("MPU 0 T: "));
  Serial.println(mpu0.getTemp());
  Serial.print(F("MPU 1 X: "));
  Serial.println(mpu1.getAccX());
  Serial.print(F("MPU 1 Y: "));
  Serial.println(mpu1.getAccY());
  Serial.print(F("MPU 1 Z: "));
  Serial.println(mpu1.getAccZ());
  Serial.print(F("MPU 1 T: "));
  Serial.println(mpu1.getTemp());
}
// ----------------------------------------------------------------------------------------------
void readMPU(char mpuNo) { // X, Y, Z, x, y, z, T
  cmd = mpuNo;
  if (mpuNo == '0' || mpuNo == '1') {
    if(Serial.available()) cmd = Serial.read();       // Get command from PC
  }
  else mpuNo = '2';                                   // Get average of both MPU0 and MPU1
  Serial.println((int)getMPU(mpuNo, cmd));
  cmd = 0;
}
// ----------------------------------------------------------------------------------------------
float getMPU(char mpu, char c) { // X, Y, Z, x, y, z, T
  float value = 0;

  if (mpu != '1') {
    mpu0.fetchData();
    switch(c) {
      case 'X': value = mpu0.getAccX();         break;
      case 'Y': value = mpu0.getAccY();         break;
      case 'Z': value = mpu0.getAccZ();         break;
      case 'x': value = mpu0.getGyroX();        break;
      case 'y': value = mpu0.getGyroY();        break;
      case 'z': value = mpu0.getGyroZ();        break;
      case 'T': value = mpu0.getTemp() - 5.1;   break;
    }
  }
  if (mpu != '0') {
    mpu1.fetchData();
    switch(c) {
      case 'X': value += mpu1.getAccX();        break;
      case 'Y': value += mpu1.getAccY();        break;
      case 'Z': value += mpu1.getAccZ();        break;
      case 'x': value += mpu1.getGyroX();       break;
      case 'y': value += mpu1.getGyroY();       break;
      case 'z': value += mpu1.getGyroZ();       break;
      case 'T': value += mpu1.getTemp() - 7.7;  break;
    }
  }
  if (mpu == '2') value /= 2;
  return value * 100;
}
// ----------------------------------------------------------------------------------------------
char scopeSafety() {    // *  1 = Safe, 0 = unsafe
  float x = getMPU('2', 'X');
  float y = getMPU('2', 'Y');
/*  Serial.print("X: "); Serial.println(x);
  Serial.print("Y: "); Serial.println(y);
  Serial.print("abs: "); Serial.println(abs(x) + abs(y));
  Serial.print("tol: "); Serial.println(TOLERANCE);
  Serial.print("safe: "); Serial.println((abs(x) + abs(y)) < TOLERANCE);*/
  return cmd = '0' + ((abs(x) + abs(y)) < TOLERANCE);
}
// ----------------------------------------------------------------------------------------------
void getMPUpos() {
  EEPROM.get(1024 - sizeof(parked), parked);                            // Get parked positions and MPU offsets
  mpu0.setAccOffsets(parked.x0off, parked.y0off, 0);
  mpu1.setAccOffsets(parked.x1off, parked.y1off, 0);
}
// ----------------------------------------------------------------------------------------------
void saveMPUpos() {     // @S  Use to save position for parked position and offsets
  write_eeMPU();
}
// ----------------------------------------------------------------------------------------------
void write_eeMPU() {
  EEPROM.put(1024 - sizeof(parked), parked);
}
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ROOF BUDDY
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void roofBuddyCmds() {              // Prefix '^' - relay commands to RoofBuddy via Bluetooth
  delay(25);
  if (!Serial.available()) return;

  if (isDigit(Serial.peek())) {
    int value = Serial.parseInt();  // command is in ASCII code to reduce chance of the command being misinterpeted as another TopBox command
    cmd = value;                    // e.g. 101 = 'e'
  }
  else cmd = Serial.read();

  if (strchr("ACDHILMOPQRUVXabcdeghilmoprswx:~/", cmd) != 0){   // Commands available for sending to RoofBuddy directly
    cmdRB();
  }
  else {
    switch (cmd)  {                                             // Commands handled by TopBox
      case '\r':                break;                          // Soak up CR character
      case '\n':                break;                          // Soak up LF (new line) character
      case '^': chkRB();        break;                          // ^~ returns '1$' if Bluetooth connection exists, '0$' if not
      case 'S': safetyRB();     break;                          // ^S returns '1' if scope is safe, '0' if not
      case '0': RBcomms = 0;    break;                          // Allow TopBox to send scope safety info to RoofBuddy
      case '1': RBcomms = 1;    break;                          // Stop TopBox sending scope safety into to RoofBuddy
      default: cmd = '%';       break;                          // Uh?
    }
  }
}
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void cmdRB() {                                    // Sends command from PC to RoofBuddy
  RBcomms = 1;
  char buf[64];
  if (Serial.peek() == '=') Serial.read();        // Skip the '=' sign
  for (byte i = 0; i < 63; i++) buf[i] = 0;       // Clear buffer
  Serial.readBytesUntil('$',buf, 63);             // Read in the rest of the command string ('$' terminated)
  while(RB.available()){                          // Clear out receive buffer
    RB.find('$');
    delay(25);
  }

  if (RBstate()) {                                // Only send if there is a Bluetooth connection
    RB.flush();
    RB.print(cmd);                                // Send command to roof buddy
    RB.print(buf);
    RB.print('$');                                // Send terminator character
    for (byte i = 0; i < 63; i++) buf[i] = 0;     // Clear buffer
    RB.readBytesUntil('$', buf, 63);              // Get response from RoofBuddy ('$' terminated)
    if (buf[0]) {
      Serial.print(buf); Serial.print('$');       // If there is one, send it to the PC
    }
    cmd = 0;
  }
  else cmd = '%';
}
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool RBstate() {
//return 1;
  return digitalRead(RB_STAT);
//return analogRead(RB_STAT) > 100;
}
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void chkRB() {                                    // See if a Bluetooth link with Roof Buddy is active
  Serial.find('$');                               // Soak up the command terminator
  Serial.print(RBstate());                        // 1 = active, 0 = not
  Serial.print('$');
  cmd = 0;
}
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void safetyRB() {                                 // Asking RoofBuddy to ask yourself if scope is safe is daft
  scopeSafety();                                  // So why not find out yourself!
  Serial.find('$');                               // Read in the '$' terminator
  Serial.print(cmd); Serial.print('$');
  cmd = 0;  
}
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void roofBuddy () {                         // Service TopBox's RoofBuddy stuff
  if (isMoving == GO) return;               // Return if FocusBuddy move is in progress
  if (!RBstate()) return;                   // Return if no connection with RoofBuddy
  if (millis() > rbTimer) {                 // If timer has expired then send scope safety info to RoofBuddy
    rbTimer = millis() + RB_PERIOD;         // Reset timer
    if (RBcomms) RB.print((scopeSafety() == '1') ? 'T' : 't');
    //Serial.println((scopeSafety() == '1') ? 'T' : 't');
  }
  char b[3] = {'h', 'h', '$'};              // Buffer for button state for debouncing, assume stopped roof
  int v;

  for (byte i = 0; i <= 1; i++) {           // See if the button press (or release) is same after 20ms
    v = analogRead(BTNS);                   // Look at analog pin which buttons are connected with resistors 
    if (v > BTN_CLOSE) b[i] = 'c';          // Look to see what button is being pressed - 'c' = close roof
    if (v > BTN_OPEN) b[i] = 'o';           // 'o' = close roof
    if (!i) delay(20);                      // Wait 20ms after first iteration only
  }
  if (b[0] != b[1]) return;                 // If the button state is not the same before and after then return
  if (b[0] == rbBtn) return;                // Return if no change from last time

  rbBtn = b[0];                             // There is a new button press (or release) so store for next time
  while (RB.available()) RB.read();         // Clear out receive buffer
  RB.write(rbBtn); RB.print('$');           // Send 'c' to close roof, 'o' to open roof or 'h' to stop roof
  RB.readBytesUntil('$', b, 2);             // Soak up response
  RBcomms = 1;                              // Comms have restarted
}
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
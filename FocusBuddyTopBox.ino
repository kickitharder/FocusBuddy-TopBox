/*
  Nano Connections
  ================
  
                  +-------+
    BT_STAT   D13 o       o D12   BT_RXD
              3V3 o       o D11~  BT_TXD
              REF o       o D10~  LED
              A0  o       o D9~   LED_GND
R   PWR_ON    A1  o       o D8    BTN_GND Gy
Be  RELAY4    A2  o       o D7    BTN2    Pe
Pe  RELAY3    A3  o       o D6~   H1_A1A    (HA_A1B connected to GND)
Gy  RELAY2    A4  o       o D5~   H2_A1A    (HB_A2B connected to GND)
W   RELAY1    A5  o       o D4    BTN1    Be
              A6  o       o D3~   F_B1B   O
              A7  o       o D2    F_B1A   R
Gn  RELAY5V   5V  o  ICSP o GND           Bn
              RST o  ooo  o RST
Bk  RELAYGND  GND o 1ooo  o RX0
              VIN o       o TX1
                  +-------+

      ICSP
      ooo      5V  D11  GND
     1ooo      D12 D13  RST
            
R      bt5V     5V
Gy     btTXD    D11
Bn     btRXD    D12
Bk     btGND    GND
Gn     btSTAT   D13
*/

#define VERSION "FocusBuddy TopBox V2.2101031 by keithrickard@hotmail.com"
// For the Arduino Nano

#define btSTAT        13
#define btRXD         12
#define btTXD         11
#define LED_GND       10
#define LED           9
#define HA_A1A        6         // Dew Heater A PWM
#define HB_A1A        5         // Dew Heater B PWM
#define F_A1A         2         // HIGH: tip is +ve
#define F_A1B         3         // HIGH: tip is -ve
#define PWR_ON        A1
#define RELAY1        A2        // PS_ON
#define RELAY2        A3        // EL Panel power
#define RELAY3        A4        // Stepper motor power
#define RELAY4        A5        // Focuser motor relay
#define BTN1          7         // Button to move focuser IN
#define BTN2          4         // Button to move focuser OUT
#define BTN_GND       8

#define BT_BAUD       38400
#define BLUETOOTH     0
#define SERIAL        1
#define EE_BACKLASH   4
#define EE_DEW_A      6
#define EE_DEW_B      7
#define EE_FOCPOSPTR  8
#define EE_PRESETS    10
#define WATCHPERIOD   3000
#define LED_DIM       4
#define LED_ON        255

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
SoftwareSerial btSerial(btTXD, btRXD);

const long MAX_LIMIT = 1000000;
byte currDir   = STOP;
byte currSpeed = STOP;
char cmd;
byte serial, dir, revPol;
byte pcWatch = 0;

byte heaters[2] = {0, 0};
const byte relayPins[4] = {RELAY1, RELAY2, RELAY3, RELAY4};
byte state[4] = {INPUT, INPUT, INPUT, INPUT};
bool isMoving, btn;
unsigned long focTimer, focStart, maxLimit, tempPos;
unsigned long pcWatchTimer = 0;
unsigned long updateTimer = 0;
long focPos, focTarg;
int backlash;
struct ee{
  long preset[10];
  char name[10][32];
} ee;

void setup() {
  pinMode(btSTAT,   INPUT);
  pinMode(LED,      OUTPUT);
  pinMode(LED_GND,  OUTPUT);
  pinMode(F_A1A,    OUTPUT);
  pinMode(F_A1B,    OUTPUT);
  pinMode(HA_A1A,   OUTPUT);
  pinMode(HB_A1A,   OUTPUT);
  pinMode(BTN_GND,  OUTPUT);
  pinMode(BTN1,     INPUT_PULLUP);
  pinMode(BTN2,     INPUT_PULLUP);
  for (byte i = 0; i <= 3; i++){
    pinMode(relayPins[i], INPUT);
  }
  analogWrite(LED,      LED_DIM);
  digitalWrite(LED_GND, LOW);
  digitalWrite(F_A1A,   LOW);
  digitalWrite(F_A1B,   LOW);
  digitalWrite(BTN_GND, LOW);
  btSerial.begin(BT_BAUD);
  btSerial.setTimeout(100);
  Serial.begin(9600);
  Serial.setTimeout(100);
  
  isMoving = STOP;
  tempPos = 0;
  btn = 0;
  eepromSetUp();
}

// ==============================================================================================
void loop() {
  command(SERIAL);                          // See if command has been received via the Serial port
  if (btSTAT) command(BLUETOOTH);           // See if command has been received via Bluetooth
  if (isMoving == GO) {
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
  if (pcWatch) if (millis() > pcWatchTimer) focuser(HALT);  // Stop focuser if PC is not watching
  
  byte btn_in = !digitalRead(BTN1);         // Act on any buttons being pressed
  byte btn_out = !digitalRead(BTN2);
  if (btn_in || btn_out) {
    if (analogRead(PWR_ON) > 511){
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
}

// ==============================================================================================
void command(byte s) {
  serial = s;
  cmd = 0;
  if (serial) if(Serial.available()) cmd = Serial.read();
  if (!serial) if(btSerial.available()) cmd = btSerial.read();
  if (!cmd) return;
  
  switch (cmd) {                                    // Process the command
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
    case 'G': pcWatch = 1; focuser(GOTO);   break;  // Go fast to given postion
    case 'g': focuser(GOTO);                break;
    case 'H': pcWatch = 1; focuser(HOME);   break;  // Go fast to home (preset 0)
    case 'h': focuser(HOME);                break;
    case 'S': pcWatch = 1; focuser(SLOT);   break;  // Goto a stored focuser position in preset 0 - 9
    case 's': focuser(SLOT);                break;
    case 'P': focuserGetPos();              break;  // Get focuser's postion
    case 'p': focuserSetPos();              break;  // Set focuser's postion
    case 'L': cmd = dir ? 'I' : 'O';        break;  // Get last direction
    case 'l': cmd = dir ? 'I' : 'O';        break;
    case 'r': clockGet();                   break;  // Get Arduino's millis() clock
    case 'R': reset();                      break;  // Soft reset arduino and its millis() timer
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
    case '?': help();                       break;
    case '~': cmd = (analogRead(PWR_ON) > 511) + '0'; break; // 1 = PSU power is on, 0 = PSU power is off.
    default: cmd = 0;                       break;  // Command is not valid so no action
  }
  if (cmd) serial ? Serial.println(cmd) : btSerial.println(cmd);  // Echo the command or '%' if there is an error
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
  
// DETERMINE HOW MOVE TO THE FOCUSER
  if (action == SLOT || action == HOME) {                                    // See if need to go to a slot preset
    int e = (action == SLOT ? (serial ? Serial.parseInt() : btSerial.parseInt()) : 0); // Get slot number or use Home position
    if (e >= 0 && e <= 9){
      EEPROM.get(EE_PRESETS, ee);
      focTarg = (ee.preset[e]);
    } else {
      cmd = '%';
      return;
    }
  } 
  else {                                                                    // Slot not selected - act accordingly
    focTarg = (serial ? Serial.parseInt() : btSerial.parseInt());           // See if a value has been provided (0 if not)
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
/*int c;
  if (revPol != digitalRead(JMPR)) {                                        // If revPol <> jumper is remove, switch polarity
    c = a;
    a = b;
    b = c;
  }*/
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
  serial ? Serial.println(position) : btSerial.println(position);
  if (pcWatch == 1) pcWatchTimer = millis() + WATCHPERIOD;
  return position;
}

// ==============================================================================================
void focuserSetPos() {
  long pos = (serial ? Serial.parseInt() : btSerial.parseInt());
  if (pos > maxLimit) pos = maxLimit;
  if (pos > 0) {
    focPos = pos;
    eepromFocPos();
  }
}

// ==============================================================================================
void targetGet() {
  cmd = 0;
  serial ? Serial.println(focTarg) : btSerial.println(focTarg);
}

// ==============================================================================================
void backlashGet() {
  cmd = 0;
  EEPROM.get(EE_BACKLASH, backlash);
  serial ? Serial.println(backlash) : btSerial.println(backlash);
}

// ==============================================================================================
void backlashSet() {
  int b = (serial ? Serial.parseInt() : btSerial.parseInt());
  if (b >= -10000 && b <= 10000) backlash = b;
  EEPROM.put(EE_BACKLASH, backlash);
}

// ==============================================================================================
void clockGet(){
  cmd = 0;
  serial ? Serial.println(millis()) : btSerial.println(millis());
}

// ==============================================================================================
void eepromTempPos() {
  long pos = (serial ? Serial.parseInt() : btSerial.parseInt());
  if (pos > 0) tempPos = pos;
}

// ==============================================================================================
void eepromGetPreset(){
  char buf[2]; 
  serial ? Serial.readBytes(buf, 1) : btSerial.readBytes(buf, 1);
  byte e = buf[0] - '0';
  if (e <= 9) {
    cmd = 0;
    EEPROM.get(EE_PRESETS, ee);
    long pos = (ee.preset[e]);
    serial ? Serial.println(pos) : btSerial.println(pos);      
  }
  else cmd = '%';
}

// ==============================================================================================
void eepromSetPreset() {
  char buf[2]; 
  serial ? Serial.readBytes(buf, 1) : btSerial.readBytes(buf, 1);
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
  serial ? Serial.readBytes(buf, 1) : btSerial.readBytes(buf, 1);
  byte e = buf[0] - '0';
  if (e <= 9) {
    EEPROM.get(EE_PRESETS, ee);
    cmd = 0;
    serial ? Serial.println(ee.name[e]) : btSerial.println(ee.name[e]);
  }
  else cmd = '%';
}

// ==============================================================================================
void eepromSetName() {
  char buf[2]; 
  serial ? Serial.readBytes(buf, 1) : btSerial.readBytes(buf, 1);
  byte e = char(buf[0]) - '0';
  if (e >= 1 && e <= 8) {                                    // 0 and 9 are not adjustable (Home and Upper Limit)
    String name = Serial.readStringUntil('#');
    EEPROM.get(EE_PRESETS, ee);
    for (byte i = 0; i <= 30; i++) {                        // Only accept 31 chars for the preset name
      ee.name[e][i] = '\0';
      ee.name[e][i + 1] = '\0';
      if (i == name.length() || name.charAt(i) == '\r' || name.charAt(i) == '\n') break;
      ee.name[e][i] = name.charAt(i);
    }
    EEPROM.put(EE_PRESETS, ee);
  }
  else {
    cmd = '%';
    if (serial) while (Serial.available()) Serial.read();   // Clear the applicable serial buffer
    else while(btSerial.available()) btSerial.read();
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
  serial ? Serial.print("Formating EEPROM... ") : btSerial.print("Formating EEPROM... ") ;
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
  EEPROM.put(EE_DEW_A, heaters[0]);                            // Save dew heater A power value
  EEPROM.put(EE_DEW_B, heaters[1]);                            // Save dew heater B power value
  EEPROM.put(EE_FOCPOSPTR, (int)(sizeof(ee) + EE_PRESETS));   // Save pointer for the current focuser position
  eepromFocPos();                                             // Save the focuser position
  textPrint("done");
}
// ==============================================================================================
void eepromWipe() {
  for (int i = 0; i <= 1023; i++) EEPROM.put(i, (byte)0xFF);
}

// ==============================================================================================
void eepromDisp() {
  byte b;
  serial ? Serial.println() : btSerial.println();
  for (int i = 0; i <= 1023; i += 8) {
    hexDigits(i >> 8);
    hexDigits(i & 0xFF);
    serial ? Serial.print(" ") : btSerial.print(" ");
    for (int j = 0; j <= 7; j++){
      hexDigits(EEPROM.read(i + j));
      serial ? Serial.print(" ") : Serial.print(" ");
    }
    for (int j = 0; j <= 7; j++) {
      b = EEPROM.read(i + j);
      if (b < 32 || b > 126) b = '.';
      serial ? Serial.print(char(b)) : Serial.print(char(b));
    }
    serial ? Serial.println() : btSerial.println();
    cmd = 0;
  }  
}
// ==============================================================================================
void hexDigits(byte b){
  if (b < 16) serial ? Serial.print('0') :btSerial.print('0');
  serial ? Serial.print(b, HEX) : btSerial.print(b, HEX);
}

// ==============================================================================================
void eepromGetPresets() {
  unsigned long check;

  cmd = '%';                          // Assume format error
  EEPROM.get(0, check);               // See if EEPROM has been formatted
  if (check != 0x0D6015EE) return;    // EE(PROM) I5 600D.  If not this then EEPROM not formatted. Quit
  
  if(serial){
    Serial.print("\nBacklash Allowance   : ");
    Serial.println(backlash);
    Serial.print("Last movement        : ");
    Serial.print(dir ? "Out" : "In");
    Serial.print("wards\nmillis()             : ");
    Serial.println(millis());
    Serial.print("Polarity on out moves: ");
    Serial.println(revPol ?  "-ve" : "+ve");
    Serial.print("\nP ");
  } else {
    btSerial.print("\nBacklash Allowance   : ");
    btSerial.println(backlash);
    btSerial.print("Last movement        : ");
    btSerial.print(dir ? "Out" : "In");
    btSerial.print("wards\nmillis()             : ");
    btSerial.println(millis());
    btSerial.print("Polarity on out moves: ");
    btSerial.println(revPol ?  "-ve" : "+ve");
    btSerial.print("\nP ");
  }
  focuserGetPos();
  for (byte i = 0; i <=9; i++) {
    if (serial) {
      Serial.print(i);
      Serial.print(' ');
      Serial.print(ee.preset[i]);
      Serial.print(' ');
      Serial.println(ee.name[i]);
    } else {
      btSerial.print(i);
      btSerial.print(' ');
      btSerial.print(ee.preset[i]);
      btSerial.print(' ');
      btSerial.println(ee.name[i]);
    }
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

  do {
    bitWrite(ptr, 15, revPol);        // Update revPol if necessary
    EEPROM.put(EE_FOCPOSPTR, ptr);    // Update ptr
    bitWrite(ptr, 15, 0);

    EEPROM.get(ptr, n);               // Get last recorded postion
    if (n == eeFocPos) break;         // If it is the same as current then return
    
    EEPROM.put(ptr, eeFocPos);        // Store the focuser position
    EEPROM.get(ptr, n);               // See if it was stored OK
    if (n == eeFocPos) break;         // Quit if it has
    ptr ++;                           // Otherwise move along to the next location

  } while (ptr <= 1020);              // Try saving the focus position again until space runs out
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
  textPrint(F("Resetting"));
  asm volatile ("jmp 0");
}

// ==============================================================================================
void switches() {   // $ command
  char buf[3];
  cmd = '%';
  if((serial ? Serial.readBytes(buf, 2) : btSerial.readBytes(buf, 2)) < 2) return;
  byte relay = buf[0] - '1';
  if (relay > 3) return;                                                  // Only 1, 2, 3, 4 are valid
  cmd = '$';
  if (buf[1] == '+') pinMode(relayPins[relay], state[relay] = OUTPUT);   // Switch to NO position
  if (buf[1] == '-') pinMode(relayPins[relay], state[relay] = INPUT);    // Switch to NC position
  if (buf[1] == '=') {
    char c = (state[relay]) == INPUT ? '-' : '+';
    serial ? Serial.println(c) : btSerial.println(c);
    cmd = 0;
  }
}
// ==============================================================================================
void heaterControls() {    // # command
  char buf[2];
  cmd = '%';
  if((serial ? Serial.readBytes(buf, 1) : btSerial.readBytes(buf, 1)) < 1) return;
  
  byte heater = buf[0] - 'A';                         // Get heater number
  if (heater > 2) return;                             // Only A or B are valid
  delay(100);
  if ((serial ? Serial.peek() : btSerial.peek())  == '=') {
    EEPROM.get(EE_DEW_A, heaters[0]);
    EEPROM.get(EE_DEW_B, heaters[1]);
    serial ? Serial.println(heaters[heater]) : btSerial.println(heaters[heater]);
    cmd = 0;
    return;
  }
  int value = (serial ? Serial.parseInt() : btSerial.parseInt());
  if (value > 255) return;
  analogWrite(heater ? HA_A1A : HB_A1A, heaters[heater] = byte(value));
  EEPROM.put(EE_DEW_A, heaters[0]);
  EEPROM.put(EE_DEW_B, heaters[1]);
  cmd = '#';
}

// ==============================================================================================
void version() {
  cmd = 0;
  textPrint(VERSION);
}

// ==============================================================================================
void help() {
  version();
  textPrint(F("\nACTION COMMANDS"));
  textPrint(F("Q q : Stop focuser immediately"));
  textPrint(F("O   : Move outwards to end of travel *"));
  textPrint(F("OX  : Move outwards for X steps (1-10000) *"));
  textPrint(F("o   : Move outwards to end of travel"));
  textPrint(F("oX  : Move outwards for X steps (1-10000)"));
  textPrint(F("I   : Move inwards to end of travel *"));
  textPrint(F("IX  : Move inwards for X steps (1-10000) *"));
  textPrint(F("i   : Move inwards to end of travel"));
  textPrint(F("iX  : Move inwards for X steps (1-10000)"));
  textPrint(F("GX  : Goto position X *"));
  textPrint(F("gX  : Goto position X"));
  textPrint(F("H   : Goto to Home postion *"));
  textPrint(F("h   : Goto to Home postion"));
  textPrint(F("SX  : Goto preset X *"));
  textPrint(F("sX  : Goto preset X"));
  textPrint(F("R   : Restart FocusBuddy and its millis() clock"));
  textPrint(F("#X= : Get Dew Band X (A or B) power value (0-255)"));
  textPrint(F("#XN : Set Dew Band X (A or B) power value N (0-255"));
  textPrint(F("$X= : Get Relay X (1-4) status: '+' = closed, '-' = open"));
  textPrint(F("$X- : Set Relay X (1-4) status to closed"));
  textPrint(F("$X+ : Set Relay X (1-4) status to open"));
  textPrint(F("~   : Senses the main power suppiy. 1 = on, 0 = off"));
  textPrint(F("\n* Note: focuser stops after 3 secs if no comms from PC"));
  textPrint(F("\nPROPERTIES"));
  textPrint(F("P   : Get focuser position"));
  textPrint(F("pX  : Set focuser position"));
  textPrint(F("T t : Get target position"));
  textPrint(F("L l : Get last move direction"));
  textPrint(F("M m : Is focuser moving? 1 = yes, 0 = no"));
  textPrint(F(">   : Default polarity for focuser motor (+)"));
  textPrint(F("<   : Reverse polarity for focuser motor (-)"));
  textPrint(F("|   : Get polarity: + or - "));
  textPrint(F("bX  : Set backlash allowance: -ve Inwards, +ve Outwards, 0 None"));
  textPrint(F("B   : Get backlash allowance"));
  textPrint(F("r   : Get Arduino's millis() clock"));
  textPrint(F("V v : Get FocusBuddy Verion"));
  textPrint(F("?   : Get this help page"));
  textPrint(F("\nEEPROM COMMANDS"));
  textPrint(F("nXY#: Set preset X (1-8) name to Y, terminated by #"));
  textPrint(F("NX  : Get preset X (0-9) name"));
  textPrint(F("Z z : Provide focuser postion for storing"));
  textPrint(F("eX  : Set preset X (1-8) position to that provided by Z command"));
  textPrint(F("Ex  : Get preset X (0-9) position"));
  textPrint(F("D d : Get all presets info and other settings"));
  textPrint(F("W w : Wipe EEPROM storage"));
  textPrint(F("F f : Format EEPROM storage structure"));
  textPrint(F("C c : Get complete contents of EEPROM in HEX"));
}

// ==============================================================================================
void textPrint(String txt) {
  serial ? Serial.println(txt) : btSerial.println(txt);
}

// ==============================================================================================
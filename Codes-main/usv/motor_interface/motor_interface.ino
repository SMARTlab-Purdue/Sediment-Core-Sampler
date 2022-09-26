uint8_t header_duo60 = 0x55;
uint8_t header_duo30 = 0x80;

// SmartDriveDuo-60 drives thrusters
// Serial1 is connected
// WARNING: Make sure the motor driver's SW004-006 are ON
uint8_t thruster_address = 0x07;

// SmartDriveDuo-30 drives winches for anchors
// Serial1 is connected
// WARNING: Make sure the motor driver's SW006 is ON
uint8_t winches_anchors_address = 0x01;

// SmartDriveDuo-30 drives winches for sampler
// Serial3 is connected
// WARNING: Make sure the motor driver's SW005 is ON
uint8_t winches_sampler_address = 0x02;

#define STRING_BUF_NUM 64
String cmd[STRING_BUF_NUM];

uint8_t packet[4];

uint8_t clutch_sampler_pin = 2;
uint8_t clutch_left_pin = 3;
uint8_t clutch_right_pin = 4;


// Left EM connections
int in1 = 8;
int in2 = 9;
// Right EM connections
int in3 = 10;
int in4 = 11;

//On-off state of EMs
bool leftEM = HIGH;
bool rightEM = HIGH;



void setup() {
  // Make clutch ON
  pinMode(clutch_sampler_pin, OUTPUT);
  pinMode(clutch_left_pin, OUTPUT);
  pinMode(clutch_right_pin, OUTPUT);
  delay(100);
  digitalWrite(clutch_sampler_pin, LOW);
  digitalWrite(clutch_left_pin, LOW);
  digitalWrite(clutch_right_pin, LOW);
  

    // Set all the motor control pins to outputs
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  
  // Turn ON all EM - Initial state (to not release the anchor immediately)
  digitalWrite(in1, HIGH);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, HIGH);
  

  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial1.begin(9600); // Initialize serial
  Serial2.begin(9600);
  Serial3.begin(9600);

  delay(2000); // Delay for 2 seconds.
  
  Serial1.write(header_duo60); // Dummy byte for auto baudrate.
  Serial2.write(header_duo30); // Dummy byte for auto baudrate.
  Serial3.write(header_duo30); // Dummy byte for auto baudrate.
  delay(3000); // Delay for 3 seconds.
}

void split(String data, char separator, String* temp)
{
  int cnt = 0;
  int get_index = 0;

  String copy = data;
  
  while(true)
  {
    get_index = copy.indexOf(separator);

    if(-1 != get_index)
    {
      temp[cnt] = copy.substring(0, get_index);

      copy = copy.substring(get_index + 1);
    }
    else
    {
      temp[cnt] = copy.substring(0, copy.length());
      break;
    }
    ++cnt;
  }
}

void make_packet(uint8_t motor_select, uint8_t address, uint8_t motor_speed) {
  packet[0] = header_duo60;
  packet[1] = ((motor_select & 0x01) << 3) | address;
  packet[2] = motor_speed;
  packet[3] = packet[0] + packet[1] + packet[2];
}


// turn on left EM
void left_EM_on() {
  // Turn on left EM
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);

  leftEM = HIGH;  //flag state EM state as ON
}

// turn off left EM
void left_EM_off() {
  if(leftEM == HIGH) {
    // reverse polarity to break "semi-permanent" magnetism
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    delay(25);
  }

  // Turn off EM
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);

  leftEM = LOW;   //flag state EM state as OFF
}

// turn on right EM
void right_EM_on() {
  // Turn on left EM
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);

  rightEM = HIGH;  //flag state EM state as ON
}

// turn off right EM
void right_EM_off() {
  if(rightEM == HIGH) {
    // reverse polarity to break "semi-permanent" magnetism
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    delay(25);
  }

  // Turn off EM
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

  rightEM = LOW;   //flag state EM state as OFF
}



void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available()) {
    String read_string = Serial.readStringUntil('\n');
    read_string.trim();
    split(read_string, ' ', cmd);

    if (cmd[0] == "th") {  // left/right thrusters
      if (cmd[1] == "l") {
        make_packet(0, thruster_address, cmd[2].toInt());
      } else if (cmd[1] == "r") {
        make_packet(1, thruster_address, cmd[2].toInt());
      }
      Serial1.write(packet, 4);
    } else if (cmd[0] == "an") { // left/right anchors
      if (cmd[1] == "l") {
        make_packet(0, winches_anchors_address, cmd[2].toInt());
      } else if (cmd[1] == "r") {
        make_packet(1, winches_anchors_address, cmd[2].toInt());
      }
      Serial2.write(packet, 4);
    } else if (cmd[0] == "cll") { // left clutch
      if (cmd[1] == "on") {
        //digitalWrite(clutch_left_pin, HIGH);
        left_EM_on();
      } else if (cmd[1] == "off") {
        //digitalWrite(clutch_left_pin, LOW);
        left_EM_off();
      }
    } else if (cmd[0] == "clr") { // right clutch
      if (cmd[1] == "on") {
        //digitalWrite(clutch_right_pin, HIGH);
        right_EM_on();
      } else if (cmd[1] == "off") {
        //digitalWrite(clutch_right_pin, LOW);
        right_EM_off();
      }
    } else if (cmd[0] == "sa") { // sampler
      if (cmd[1] == "l") {
        make_packet(0, winches_sampler_address, cmd[2].toInt());
      } else if (cmd[1] == "r") {
        make_packet(1, winches_sampler_address, cmd[2].toInt());
      }
      Serial3.write(packet, 4);
    } else if (cmd[0] == "cl") { // sampler clutch
      if (cmd[1] == "on") {
        digitalWrite(clutch_sampler_pin, HIGH);
      } else if (cmd[1] == "off") {
        digitalWrite(clutch_sampler_pin, LOW);
      }
    } else if (cmd[0] == "test") {
      Serial1.write(85);
      Serial1.write(7);
      Serial1.write(63);
      Serial1.write(85 + 7 + 63);
      delay(2000);
      Serial1.write(85);
      Serial1.write(7);
      Serial1.write(192);
      Serial1.write(85 + 7 + 192);
      delay(2000);
      Serial1.write(85);
      Serial1.write(7);
      Serial1.write(127);
      Serial1.write(85 + 7 + 127);
      delay(2000);
    } else if (cmd[0] == "init") {
      Serial1.write(header_duo60);
      Serial2.write(header_duo30);
      Serial3.write(header_duo30);
    }
  }
}

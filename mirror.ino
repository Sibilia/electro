#include <TimerOne.h>

#define foto_pin1 2
#define foto_pin2 1
#define ir_led_pin1 10
#define ir_led_pin2 11
#define heat_pin 8
#define led_pin 9

bool ir1_status;
bool prev_ir1_status;
bool ir2_status;
bool prev_ir2_status;
bool led_status;
bool heat_status;

unsigned long starttime = 0;
unsigned long heat_delay = 1000 * 60 * 15; // 15 minut

int foto_threshold = 50;

void setup() {
  pinMode(ir_led_pin1, OUTPUT);
  pinMode(led_pin, OUTPUT);
  pinMode(ir_led_pin2, OUTPUT);
  pinMode(heat_pin, OUTPUT);
  pinMode(foto_pin1, INPUT);
  pinMode(foto_pin2, INPUT);

  Timer1.initialize(100);
  Timer1.pwm(led_pin, 0);

  led_status = false;
  heat_status = false;
  prev_ir1_status = false;
  prev_ir2_status = false;
  
  digitalWrite(heat_pin, LOW);
  digitalWrite(ir_led_pin1, LOW);
  digitalWrite(ir_led_pin2, LOW);

  Serial.begin(9600);
  Serial.println("Starting.");
}

void loop() {

  unsigned long currentTime = 0;
  long heat_time = 0;

  Serial.println("check heat irled.");
  ir1_status = check_ir(foto_pin1, ir_led_pin1);
  Serial.println("check led irled.");
  ir2_status = check_ir(foto_pin2, ir_led_pin2);

  if ( ir1_status and not heat_status and not prev_ir1_status ) {
    Serial.println("heat on.");
    digitalWrite(heat_pin, HIGH);
    heat_status = true;
    starttime = millis();

  } else if ( ir1_status and heat_status and not prev_ir1_status ) {
    Serial.println("heat irled off.");
    heat_off();

  } else if ( heat_status ) {
    currentTime = millis();
  
    if ( starttime > currentTime ) {
      heat_time = currentTime - starttime;
    } else {
      heat_time = 4294967295 - starttime + currentTime;
    }
    Serial.print("heat timer:");
    Serial.println(heat_time);
    
    if ( heat_time > heat_delay ){
      Serial.println("heat timer off.");
      heat_off();
    }
  }

  if ( ir1_status and not prev_ir1_status ) {
    prev_ir1_status = true;
  } else if ( not ir1_status and prev_ir1_status ) {
    prev_ir1_status = false;
  }
  
  if ( ir2_status and not led_status and not prev_ir2_status ) {
    Serial.println("Led on...");
    lightOnOff (1);
    led_status = true;
  } else if ( ir2_status and led_status and not prev_ir2_status ) {
    Serial.println("Led off...");
    lightOnOff (0);
    led_status = false;
  }

  if ( ir2_status and not prev_ir2_status ) {
    prev_ir2_status = true;
  } else if ( not ir2_status and prev_ir2_status ) {
    prev_ir2_status = false;
  }
  
  Serial.println("-----------");
  delay(250);
} 

bool check_ir(int irf_pin, int irl_pin) {
  int temp_val[4];
  int val = 0;
  
  for (int i=0; i<5; i++){
    digitalWrite(irl_pin, HIGH);
    delay(2);
    val = analogRead(irf_pin); 
    digitalWrite(irl_pin, LOW); 
    delay(2);
    temp_val[i] = analogRead(irf_pin) - val;
    Serial.println( temp_val[i] );
  }

  Serial.println( "-----" );

  if ( temp_val[0] > foto_threshold and temp_val[1] > foto_threshold and temp_val[2] > foto_threshold and temp_val[3] > foto_threshold and temp_val[4] > foto_threshold) {
    return true;
    Serial.println("foto open");
  } else {
    return false;
  }
}

void lightOnOff (int state) {
  int xdelay = 2000 / 1023;

  if ( state == 1 ) {
    for (int i = 0; i <= 1023; i++) {
      Timer1.setPwmDuty(led_pin, i);
      delay(xdelay);
    }
  } else {
    for (int i = 1023; i >= 0; i--) {
      Timer1.setPwmDuty(led_pin, i);
      delay(xdelay);
    }
  }
}

void heat_off () {
  digitalWrite(heat_pin, LOW);
  heat_status = false;
}


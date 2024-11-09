#include <Arduino.h>

#define ARM_MATH_CM4
#include <arm_math.h>
#define HWSERIAL Serial3  // UART

#define PWM_RSF 2  // PWM shoulder right front
#define PWM_REF 3  // PWM elbow right front
#define PWM_RSB 4  // PWM shoulder right back
#define PWM_REB 5  // PWM elbow right back
#define PWM_LSF 6  // PWM shoulder left front
#define PWM_LEF 7  // PWM elbow left front
#define PWM_LSB 8  // PWM shoulder left back
#define PWM_LEB 9  // PWM elbow left back

char greeting = 1;  // The program welcomes
char control = 0;   // 1 begin control loop

const int down[8] = {10 /*RSF*/, 70, 10, 70, 165 /*LSF*/, 170, 165, 170};


/*
int p1 = 1600, new_angle1 = 0, old_angle1 = 0, e1 = 0;
int p2 = 1600, new_angle2 = 0, old_angle2 = 0, e2 = 0;
int p3 = 1600, new_angle3 = 0, old_angle3 = 0, e3 = 0;
int p4 = 1600, new_angle4 = 0, old_angle4 = 0, e4 = 0;
int p5 = 1600, new_angle5 = 0, old_angle5 = 0, e5 = 0;
int p6 = 1600, new_angle6 = 0, old_angle6 = 0, e6 = 0;
int p7 = 1600, new_angle7 = 0, old_angle7 = 0, e7 = 0;
int p8 = 1600, new_angle8 = 0, old_angle8 = 0, e8 = 0;
*/

//down
int p1 = 1970, new_angle1 = 0, old_angle1 = 10, e1 = 0;
int p2 = 4190, new_angle2 = 0, old_angle2 = 70, e2 = 0;
int p3 = 1970, new_angle3 = 0, old_angle3 = 10, e3 = 0;
int p4 = 4190, new_angle4 = 0, old_angle4 = 70, e4 = 0;
int p5 = 7705, new_angle5 = 0, old_angle5 = 165, e5 = 0;
int p6 = 7830, new_angle6 = 0, old_angle6 = 145, e6 = 0;
int p7 = 7705, new_angle7 = 0, old_angle7 = 165, e7 = 0;
int p8 = 7830, new_angle8 = 0, old_angle8 = 180, e8 = 0;


int pmin = 1600;
int pmax = 8200;

/*   UART setup              */
unsigned char bytecount = 0;
unsigned char received_data[0x08] = { 0 };
unsigned char p = 0;
unsigned char i = 0;


volatile bool ledState = false;  // from ChatGPT

void setup() {
  // from ChatGPT begin
  pinMode(LED_BUILTIN, OUTPUT);

  // Stop the timers before configuration
  PIT_MCR = 0x00;  // Enable PIT and disable freezing in the debugger

  // PIT0 configuration
  PIT_LDVAL0 = 200000 - 1;  // Set timer period (1 second at 120 MHz)
  PIT_TCTRL0 = 0x3;         // Enable timer and interrupt

  // Assign interrupt function
  attachInterruptVector(IRQ_PIT, pit0_isr);
  NVIC_ENABLE_IRQ(IRQ_PIT);
  // end ChatGPT


// UART
Serial3.begin(115200);
Serial.begin(115200);

// PWM Setup
analogWriteResolution(16);  // analogWrite value 0 to 4095, or 4096 for high
// analogWriteResolution(10);  // analogWrite value 0 to 1023, or 1023 for high

// sample  analogWriteFrequency(2, 10000);   // PWM frequency 10kHZ
analogWriteFrequency(2, 50);  // PWM frequency 50 HZ
analogWriteFrequency(3, 50);  // PWM frequency 50 HZ
analogWriteFrequency(4, 50);  // PWM frequency 50 HZ
analogWriteFrequency(5, 50);  // PWM frequency 50 HZ
analogWriteFrequency(6, 50);  // PWM frequency 50 HZ
analogWriteFrequency(7, 50);  // PWM frequency 50 HZ
analogWriteFrequency(8, 50);  // PWM frequency 50 HZ
analogWriteFrequency(9, 50);  // PWM frequency 50 HZ

/* PinMode setup */

  pinMode(PWM_RSF, OUTPUT);
  pinMode(PWM_REF, OUTPUT);
  pinMode(PWM_RSB, OUTPUT);
  pinMode(PWM_REB, OUTPUT);
  pinMode(PWM_LSF, OUTPUT);
  pinMode(PWM_LEF, OUTPUT);
  pinMode(PWM_LSB, OUTPUT);
  pinMode(PWM_LEB, OUTPUT);


// test
//  analogWrite(PWM_RSF, 3300); = 1.007ms
// analogWrite(PWM_RSF, 2950);  // 900ms
// analogWrite(PWM_REB, 6900);  // 2100ms
//  analogWrite(PWM_RSF, 1600); //8200
//  analogWrite(PWM_REF, 4950);
//  analogWrite(PWM_RSB, 6050);
//  analogWrite(PWM_REB, 6900);  // 2100ms

}

void loop() 
{
  // Main loop is empty, all work is done in the interrupt
// The program welcomes
if (greeting == 1) {
  Serial.println("Teensy 4.0 RS motors control v000 ");
  


analogWrite(PWM_RSF, p1);
analogWrite(PWM_REF, p2);
analogWrite(PWM_RSB, p3);
analogWrite(PWM_REB, p4);


analogWrite(PWM_LSF, p5);
analogWrite(PWM_LEF, p6);
analogWrite(PWM_LSB, p7);
analogWrite(PWM_LEB, p8);    

  greeting = 0;
}

/* read data from UART */
while (Serial3.available() && bytecount < 0x08) {
  received_data[bytecount] = Serial3.read();
  Serial.println(received_data[bytecount], HEX);
  bytecount++;
}

if (bytecount == 0x08) {

  new_angle1 = received_data[0x00];
  new_angle2 = received_data[0x01];
  new_angle3 = received_data[0x02];
  new_angle4 = received_data[0x03];
  new_angle5 = received_data[0x04];
  new_angle6 = received_data[0x05];
  new_angle7 = received_data[0x06];
  new_angle8 = received_data[0x07];

/*
  pnew1 = pmin + (received_data[0x00] * 37);
  pnew2 = pmin + (received_data[0x01] * 37);
  pnew3 = pmin + (received_data[0x02] * 37);
  pnew4 = pmin + (received_data[0x04] * 37);
  pnew6 = pmin + (received_data[0x05] * 37);
  pnew7 = pmin + (received_data[0x06] * 37);
  pnew8 = pmin + (received_data[0x07] * 37);
*/
  p = 1;
  bytecount = 0;
  control = 1;
}

if (p == 1) {
  for (i = 0; i < 0x08; i++) {
    //Serial.println(received_data[i], HEX);  // number, base 16/hexadecimal
    //Serial.println(p1, DEC);                // number, base 16/hexadecimal
  }

  // Serial1.print("M_En = ");
  // Serial.println(motors, DEC);  // number, base 16/hexadecimal
  p = 0;
}
}



void pit0_isr() 
{
    if (PIT_TFLG0 & 1)   // Check the interrupt flag for PIT0
    {  

      if(control == 1) 
      {
// Control loop for RC motor 1
if (old_angle1 != new_angle1) {
  e1 = old_angle1 - new_angle1;
  if (e1 < 0) {
    old_angle1 = old_angle1 + 1;
  } else {
    old_angle1 = old_angle1 - 1;
  }
  p1 = pmin + (old_angle1 * 37);
  analogWrite(PWM_RSF, p1);
} else {
  old_angle1 = new_angle1;
}

// Control loop for RC motor 2
if (old_angle2 != new_angle2) {
  e2 = old_angle2 - new_angle2;
  if (e2 < 0) {
    old_angle2 = old_angle2 + 1;
  } else {
    old_angle2 = old_angle2 - 1;
  }
  p2 = pmin + (old_angle2 * 37);
  analogWrite(PWM_REF, p2);
} else {
  old_angle2 = new_angle2;
}

// Control loop for RC motor 3
if (old_angle3 != new_angle3) {
  e3 = old_angle3 - new_angle3;
  if (e3 < 0) {
    old_angle3 = old_angle3 + 1;
  } else {
    old_angle3 = old_angle3 - 1;
  }
  p3 = pmin + (old_angle3 * 37);
  analogWrite(PWM_RSB, p3);
} else {
  old_angle3 = new_angle3;
}

// Control loop for RC motor 4
if (old_angle4 != new_angle4) {
  e4 = old_angle4 - new_angle4;
  if (e4 < 0) {
    old_angle4 = old_angle4 + 1;
  } else {
    old_angle4 = old_angle4 - 1;
  }
  p4 = pmin + (old_angle4 * 37);
  analogWrite(PWM_REB, p4);
} else {
  old_angle4 = new_angle4;
}

// Control loop for RC motor 5
if (old_angle5 != new_angle5) {
  e5 = old_angle5 - new_angle5;
  if (e5 < 0) {
    old_angle5 = old_angle5 + 1;
  } else {
    old_angle5 = old_angle5 - 1;
  }
  p5 = pmin + (old_angle5 * 37);
  analogWrite(PWM_LSF, p5);
} else {
  old_angle5 = new_angle5;
}

// Control loop for RC motor 6
if (old_angle6 != new_angle6) {
  e6 = old_angle6 - new_angle6;
  if (e6 < 0) {
    old_angle6 = old_angle6 + 1;
  } else {
    old_angle6 = old_angle6 - 1;
  }
  p6 = pmin + (old_angle6 * 37);
  analogWrite(PWM_LEF, p6);
} else {
  old_angle6 = new_angle6;
}

// Control loop for RC motor 7
if (old_angle7 != new_angle7) {
  e7 = old_angle7 - new_angle7;
  if (e7 < 0) {
    old_angle7 = old_angle7 + 1;
  } else {
    old_angle7 = old_angle7 - 1;
  }
  p7 = pmin + (old_angle7 * 37);
  analogWrite(PWM_LSB, p7);
} else {
  old_angle7 = new_angle7;
}

// Control loop for RC motor 8
if (old_angle8 != new_angle8) {
  e8 = old_angle8 - new_angle8;
  if (e8 < 0) {
    old_angle8 = old_angle8 + 1;
  } else {
    old_angle8 = old_angle8 - 1;
  }
  p8 = pmin + (old_angle8 * 37);
   analogWrite(PWM_LEB, p8);
} else {
  old_angle8 = new_angle8;
}

/*     
analogWrite(PWM_RSF, p1);
analogWrite(PWM_REF, p2);
analogWrite(PWM_RSB, p3);
analogWrite(PWM_REB, p4);
analogWrite(PWM_LSF, p5);
analogWrite(PWM_LEF, p6);
analogWrite(PWM_LSB, p7);
analogWrite(PWM_LEB, p8);
*/

    }

        PIT_TFLG0 = 1;         // Clear the interrupt flag
        ledState = !ledState;  // Toggle the LED state
        digitalWrite(LED_BUILTIN, ledState);
    }
}

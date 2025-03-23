

#include <avr/io.h>

#define CPU_clock 16000000
#define GYRO_CONFIG 27
#define FS_SEL0 3
#define FS_SEL1 4
#define GYRO_X_H 67
#define GYRO_X_L 68
#define GYRO_Y_H 69
#define GYRO_Z_H 71
#define ACCEL_Z_H 63
#define ACCEL_Y_H 61
#define ACCEL_X_H 59
#define SMPRT_DIV 25
#define CONFIG 26
#define SEND_START_CONDITION 100
#define SEND_START_CONDITION_AND_SET_TWINT 228
#define TWCR_INITIALISE 68
#define SET_TWINT 196
#define CLEAR_TWEA_FOR_NACK_AND_SET_TWINT 132
#define SEND_STOP_CONDITION 212
#define SLAVE_ADDRESS 104
#define STN 6
#define OFFSET -530

uint8_t IsrExitFlow;
uint8_t isrFunction;
uint8_t myRegister;
float gyroX;
float gyroY;
float gyroZ;
float accelX;
float accelY;
float accelZ;

//  Initialise the TWI peripheral
void twiInitialise(uint8_t bitRateGenerator) {

  // Activate internal pullups for twi
  digitalWrite(SDA, 1);
  digitalWrite(SCL, 1);

  TWCR = TWCR_INITIALISE;  // Setting control register bits

  TWBR = bitRateGenerator;  // Setting TWBR to 18 for a SCL frequency of 100kHz

  TWSR &= !(1 << TWPS1) & !(1 << TWPS0);  // Setting pre scaler bits to zero (Pre scaler = 1)

  //Serial.println("Initialised");
}


void writeMPU(uint8_t registerToWrite, uint8_t valueToWrite) {

  while (1) {

    // While TWINT is 0 wait in this loop
    while (!(TWCR & (1 << TWINT))) {
      ;
    }

    TWCR = TWCR_INITIALISE;  // Set TWINT to clear interrupt

    switch (TWSR) {

      case 8:
        //  Start condition has been transmitted
        //Serial.println("TWSR reads 8");
        TWCR = TWCR_INITIALISE;

        TWDR = (SLAVE_ADDRESS << 1);  // Load SLA + W

        break;

      case 16:
        // A repeated start condition has been transmitted

        break;

      case 24:
        // SLA+W has been transmitted; ACK has been received

        //Serial.println("SLA+W has been transmitted; ACK has been received, sending RA");

        TWDR = registerToWrite;  // Load register address (107)

        break;

      case 32:
        // SLA+W has been transmitted; NOT ACK has been received

        break;

      case 40:
        // Data byte has been transmitted; ACK has been received

        if (myRegister & (1 << STN)) {
          IsrExitFlow = 3;
          isrFunction = 1;
          break;
        }

        //Serial.println("Data byte has been transmitted; ACK has been received, sending '9'");

        TWDR = valueToWrite;  // Load decimal 9 into register 107 to clear sleep bit, disable temperature sensor and select Gyro X clock

        myRegister |= (1 << STN);  // stop_now++;

        break;

      case 48:
        // Data byte has been transmitted; NOT ACK has been received

        break;

      case 56:
        // Arbitration lost in SLA+W or data bytes

        break;

      case 64:
        // SLA+R has been transmitted; ACK has been received, data byte will be received and ACK will be returned

        break;

      case 80:
        // Data byte has been received; ACK has been returned, data byte will be stored and NACK will be returned

        break;

      case 88:
        // Data byte has been received; NOT ACK has been returned, data byte will be store and STOP condition will be sent to end transmission

        break;

      default:
        break;
    }

    switch (IsrExitFlow) {

      case 0:
        TWCR = SET_TWINT;  // 0b11000101
        break;

      case 1:
        //Serial.println("Repeated start");
        TWCR = SEND_START_CONDITION_AND_SET_TWINT;
        break;

      case 2:
        //Serial.println("Return NACK");
        TWCR = CLEAR_TWEA_FOR_NACK_AND_SET_TWINT;

        //gyroValue = (TWDR);

        //Serial.print("High byte stored in gyroValue: ");
        //Serial.println(gyroValue);
        break;

      case 3:
        //Serial.println("STOP condition will be sent");
        TWCR = SEND_STOP_CONDITION;
        return;
        break;

      default:
        break;
    }
  }
}


int16_t readMPU(uint8_t registerToRead) {

  int16_t readValue;

  // While communication with gyro device bit is set
  while (1) {

    // While TWINT is 0 wait in this loop
    while (!(TWCR & (1 << TWINT))) {
      ;
    }

    IsrExitFlow = 0;

    TWCR = TWCR_INITIALISE;  // Set TWINT to clear interrupt


    // Read from GYRO_X
    switch (TWSR) {

      case 8:
        //  Start condition has been transmitted
        //Serial.println("TWSR reads 8");
        TWCR = TWCR_INITIALISE;

        TWDR = (SLAVE_ADDRESS << 1);  // Load SLA + W
        break;

      case 16:
        // A repeated start condition has been transmitted

        TWDR = ((SLAVE_ADDRESS << 1) + 1);  // Load SLA + R
        //Serial.println(TWDR);
        break;

      case 24:
        // SLA+W has been transmitted; ACK has been received
        TWDR = registerToRead;  // Write the gyro data register address to the slave
        break;

      case 32:
        // SLA+W has been transmitted; NOT ACK has been received
        break;

      case 40:
        // Data byte has been transmitted; ACK has been received
        IsrExitFlow = 1;  // Exit ISR with start condition (Repeated START)
        break;

      case 48:
        // Data byte has been transmitted; NOT ACK has been received
        break;

      case 56:
        // Arbitration lost in SLA+W or data bytes
        break;

      case 64:
        // SLA+R has been transmitted; ACK has been received, data byte will be received and ACK will be returned

        // IsrExitFlow = 0;

        break;

      case 80:
        // Data byte has been received; ACK has been returned, data byte will be stored and NACK will be returned

        //Serial.print("TWDR value at supposed data receival: ");
        //Serial.println(TWDR);

        IsrExitFlow = 2;  // Return NACK

        break;

      case 88:
        // Data byte has been received; NOT ACK has been returned, data byte will be store and STOP condition will be sent to end transmission

        //gyroValue += ((uint16_t) (TWDR << 8));

        readValue = readValue << 8;

        readValue += TWDR;

        IsrExitFlow = 3;

        break;

      default:
        break;
    }


    switch (IsrExitFlow) {

      case 0:
        TWCR = SET_TWINT;  // 0b11000101
        break;

      case 1:
        //Serial.println("Repeated start");
        TWCR = SEND_START_CONDITION_AND_SET_TWINT;
        break;

      case 2:
        //Serial.println("Return NACK");
        TWCR = CLEAR_TWEA_FOR_NACK_AND_SET_TWINT;

        readValue = (TWDR);

        //Serial.print("High byte stored in gyroValue: ");
        //Serial.println(gyroValue);
        break;

      case 3:
        //Serial.println("STOP condition will be sent");
        TWCR = SEND_STOP_CONDITION;

        // readValue /= 10;
        readValue -= OFFSET;

        readValue = ((float)readValue / 32767) * 250;

        return readValue;

        break;

      default:
        break;
    }
  }

}




void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);

  while(!Serial);

  Serial.println("Serial begun");

  TWCR = SEND_START_CONDITION;
  writeMPU(107, 9); // Initialise the MPU

}

void loop() {
  // put your main code here, to run repeatedly:

  delay(100);


  // While button is being held {
    // Read from gyroscope and accelerometer XYZ

    // Read gyro X
    TWCR = SEND_START_CONDITION;
    gyroX = readMPU(GYRO_X_H);
    Serial.print(gyroX);
    Serial.print(" ");

    // Store gyro X in gyro X array

    // Read gyro Y
    TWCR = SEND_START_CONDITION;
    gyroY = readMPU(GYRO_Y_H);
    Serial.print(gyroY);
    Serial.print(" ");

    // Store gyro Y in gyro Y array

    // Read gyro Z
    TWCR = SEND_START_CONDITION;
    gyroZ = readMPU(GYRO_Z_H);
    Serial.print(gyroZ);
    Serial.print(" ");

    // Store in gyro Z array

    // Read accel X
    TWCR = SEND_START_CONDITION;
    accelX = readMPU(ACCEL_X_H);
    Serial.print(accelX);
    Serial.print(" ");
    // Store

    // Read accel Y
    TWCR = SEND_START_CONDITION;
    accelY = readMPU(ACCEL_Y_H);
    Serial.print(accelY);
    Serial.print(" ");

    // Store

    // Read accel Z
    TWCR = SEND_START_CONDITION;
    accelZ = readMPU(ACCEL_Z_H);
    Serial.print(accelZ);
    Serial.println(" ");
    
    // Store

    // delay(2);  // So for a 3 second 'gesture' there would be approximately 1500*6 = 9000 data points where in a 2D image format would be ~95x95
  // }

  // When button is released
    // Print arrays
    // Clear arrays

}

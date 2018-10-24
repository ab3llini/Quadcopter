/*
 * Library inlcudes
 */

#include <String.h>
#include <Servo.h>
#include <SoftwareSerial.h>

/*
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * COMPILER DEFINITIONS
 */

/*
 * Serial interfaces definitions (Hardware)
 */

#define MAIN_COM Serial
#define RADIO_COM Serial1
#define IMU_COM Serial2

/*
 * Serial interfaces baud rate setup
 */

#define MAIN_BAUD 115200
#define RADIO_BAUD 57600
#define IMU_BAUD 57600

/* 
 * Radio protocol definitions
 */

#define RADIO_SYNCH_BYTE -100

/*
 * Air-To-Ground setup
 */

#define TX_DELAY_MS 100

/*  
 * Pinout setup for electronic speed controllers
 */

#define ESC_NW 12
#define ESC_NE 11
#define ESC_SE 10
#define ESC_SW 9

/*
 * Electronic speed controllers definitions
 */

#define ESC_MIN_MICROS 1000
#define ESC_MAX_MICROS 1450
#define ESC_ARM_DELAY 2000

/*
 * Input data range & definitions
 */

#define INPUT_ANGLE_MAX 35
#define INPUT_ANGLE_MIN -35
#define INPUT_PWM_MAX ESC_MAX_MICROS
#define INPUT_PWM_MIN ESC_MIN_MICROS

/*
 * Proportional-Integrate-Derivative (PID) constants definitons & setup
 */

#define PITCH_P_VAL 2
#define PITCH_I_VAL 0
#define PITCH_D_VAL 0
#define ROLL_P_VAL 2
#define ROLL_I_VAL 0
#define ROLL_D_VAL 0

/*
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * TYPE DEFINITIONS
 */

/*
 * Electronic speed controller type, contains attach pin, relative pwm, Servo object to drive it
 */

typedef struct {
	int pin;
	int pwm;
	Servo esc;
} esc_t;

/*
 * Radio input typedef, synch-byte + 3 * uint8_t + 1 * 1 * int => 1 + 5bytes
 */

typedef struct {
    int8_t yaw;
    int8_t pitch;
    int8_t roll;
    int pwm;
} radio_data_t;

/*
 * Useful to drive the radio rx process
 */

typedef struct {
    unsigned long delayTx;
    int bytesRecived;
    byte buffer[sizeof(radio_data_t)];
} radio_link_t;

/*
 * Radio parent structure
 */

typedef struct {
    radio_data_t data;
    radio_link_t datalink;
} radio_t;

/*
 * IMU Input, 12byte (Processed from ASCII string through atof)
 */

typedef struct {
    float yaw;
    float pitch;
    float roll;
} imu_data_t;

/*
 * Useful to drive the IMU rx process
 */

typedef struct {
    int bytesRecived; //Quanti bytes ho ricevuto, lo uso come counter del buffer
    int ypr; //Dove salvo i dati
    char buffer[10];
} imu_link_t;

/*
 * IMU parent structure
 */

typedef struct {
    imu_link_t imulink;
    imu_data_t data;
} imu_t;

/*
 * Proportional-Integrate-Derivative (PID) type definiton
 */

typedef struct {
    float lastError;
    float errorSum;
    float output;
    unsigned long interval;
} pid_t;


/*
 * Global variables declaration
 */

// * Electronic-Speed-Controller
esc_t M_NW,M_NE,M_SW,M_SE;

// * Radio
radio_t Radio;

// * IMU
imu_t IMU;

// * PIDs
pid_t rollPID, pitchPID;

/*
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * PROGRAM BEGIN
 */

/*
 * Setup prototypes
 */

void setupRadio();
void setupIMU();
void setupPIDs();
void setupEngines();
void armEngines();

/*
 * Loop protoypes
 */

int checkRxData();

void receiveData();
void sendData();
void parseIMUData();
void updateEngines();
void computePID(float realAngle, float desiredAngle, float * output, float * lastError, float * errorSum, unsigned long * interval, float kP, float kI, float kD);

/*
 * Main setup
 */

void setup() {

    /* Begin radio communication */
    setupRadio();

    /* Begin IMU communication */
    setupIMU();

    /* PIDs Initialization */
    setupPIDs();

    /* Engines setup & arming */
    setupEngines();
    armEngines();

}

/*
 * Main loop
 */

void loop() {

    receiveData();
    parseIMUData();
    computePID(IMU.data.roll, Radio.data.roll, &(rollPID.output), &(rollPID.lastError), &(rollPID.errorSum), &(rollPID.interval), ROLL_P_VAL, ROLL_I_VAL, ROLL_D_VAL);
    computePID(IMU.data.pitch, Radio.data.pitch, &(pitchPID.output), &(pitchPID.lastError), &(pitchPID.errorSum), &(pitchPID.interval), PITCH_P_VAL, PITCH_I_VAL, PITCH_D_VAL);
    updateEngines();
}

/*
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * FUNCTION IMPLEMENTATION
 */

void setupRadio() {

    RADIO_COM.begin(RADIO_BAUD); 

    Radio.data.yaw = 0;
    Radio.data.pitch = 0;
    Radio.data.roll = 0;
    Radio.data.pwm = ESC_MIN_MICROS;

    Radio.datalink.bytesRecived = -1;
    Radio.datalink.delayTx = millis();
}

void setupIMU() {

    IMU_COM.begin(IMU_BAUD);

    IMU.data.yaw = 0;
    IMU.data.pitch = 0;
    IMU.data.roll = 0;

    IMU.imulink.bytesRecived = 0;
    IMU.imulink.ypr = 0;
}

void setupPIDs() {

    /* Pitch PID setup */
    pitchPID.errorSum = 0.0;
    pitchPID.lastError = 0.0;
    pitchPID.output = 0.0;
    pitchPID.interval = millis();

    /* Roll PID setup */
    rollPID.errorSum = 0.0;
    rollPID.lastError = 0.0;
    rollPID.output = 0.0;
    rollPID.interval = millis();
}

void setupEngines() {

    /* Attach esc */
    M_NW.esc.attach(ESC_NW);
    M_NE.esc.attach(ESC_NE);
    M_SW.esc.attach(ESC_SW);
    M_SE.esc.attach(ESC_SE);

    M_NW.pwm = ESC_MIN_MICROS;
    M_NE.pwm = ESC_MIN_MICROS;
    M_SW.pwm = ESC_MIN_MICROS;
    M_SE.pwm = ESC_MIN_MICROS;

}

void armEngines() {

    /* Delay then arm */
    delay(ESC_ARM_DELAY);

    M_NW.esc.writeMicroseconds(ESC_MIN_MICROS);
    delay(5);
    M_NE.esc.writeMicroseconds(ESC_MIN_MICROS);
    delay(5);
    M_SW.esc.writeMicroseconds(ESC_MIN_MICROS);
    delay(5);
    M_SE.esc.writeMicroseconds(ESC_MIN_MICROS);
    delay(5);
}

void receiveData() {

    if (RADIO_COM.available() > 0) {

        //Read one byte at time
        int8_t rx_byte = RADIO_COM.read();

        if (Radio.datalink.bytesRecived == -1) {

            //Until the synch-byte is found, counter is not incremented and bytes are thrown
            if (rx_byte == RADIO_SYNCH_BYTE) {

             	//Next loop will begin reading data
             	//Counter set to 0 to begin
                Radio.datalink.bytesRecived = 0;

            }

        }
        else {

            //Reading bytes and placing then in the buffer
            Radio.datalink.buffer[Radio.datalink.bytesRecived] = rx_byte;
            Radio.datalink.bytesRecived++;

        }
        
        
    }

    if (Radio.datalink.bytesRecived == sizeof(radio_data_t)) {

        //If all bytes were received
        if (checkRxData() == 1) {

            //After a brief check a memcpy is done to save them
            memcpy(&Radio.data, &Radio.datalink.buffer[0], sizeof(radio_data_t));

        }

        //Counter reset
        Radio.datalink.bytesRecived = -1;

    }

}

void sendData() {

    //Now
    unsigned long now = millis();

    //If a precise amount of time has passed, a package is sent back to ground
    if (now - Radio.datalink.delayTx  > TX_DELAY_MS && Radio.datalink.bytesRecived == 0) {

        byte payload[sizeof(imu_data_t)];

        memcpy(&payload[0], &(IMU.data), sizeof(imu_data_t));

        RADIO_COM.write(payload, sizeof(imu_data_t));

        //Resetto il delay
        Radio.datalink.delayTx = millis();

    }
}

int checkRxData() {

    radio_data_t tmp_data;

    memcpy(&tmp_data, &Radio.datalink.buffer[0], sizeof(radio_data_t));

    if (tmp_data.yaw < INPUT_ANGLE_MIN || tmp_data.yaw > INPUT_ANGLE_MAX) {
        return 0;
    }
    if (tmp_data.pitch < INPUT_ANGLE_MIN || tmp_data.pitch > INPUT_ANGLE_MAX) {
        return 0;
    }
    if (tmp_data.roll < INPUT_ANGLE_MIN || tmp_data.roll > INPUT_ANGLE_MAX) {
        return 0;
    }
    if (tmp_data.pwm < INPUT_PWM_MIN || tmp_data.pwm > INPUT_PWM_MAX) {
        return 0;
    }

    return 1;
}

void parseIMUData() {

    //Stop execution until every byte has been read
    while (IMU_COM.available()) {

        //Byte ready to be read
        char byte;

        //Float to be saved
        float newValue = 0.0;

        //Reading byte
        byte = IMU_COM.read();

        //Byte processing
        switch (byte) {
            case '0' :
            case '1' :
            case '2' :
            case '3' :
            case '4' :
            case '5' :
            case '6' :
            case '7' :
            case '8' :
            case '9' :
            case '.' :
            case '-' :
                IMU.imulink.buffer[IMU.imulink.bytesRecived] = byte;
                IMU.imulink.bytesRecived++;
                break;
            case ',' :
                //Ho finito di leggere un numero dalla fifo
                //Termino il buffer
                IMU.imulink.buffer[IMU.imulink.bytesRecived] = '\0';

                //Reimposto il contatore del buffer
                IMU.imulink.bytesRecived = 0;

                //Converto il carattere
                newValue = atof(IMU.imulink.buffer);

                //Salvo il nuovo valore nella struttura dati
                switch (IMU.imulink.ypr) {
                    case 0:
                        IMU.data.yaw = newValue;
                        IMU.imulink.ypr++;
                        break;
                    case 1:
                        IMU.data.pitch = newValue;
                        break;
                }
                break;
            case ' ' :
            case '\n':

                //Sono arrivato alla fine

                //Termino il buffer
                IMU.imulink.buffer[IMU.imulink.bytesRecived] = '\0';

                //Converto il carattere a float
                newValue = atof(IMU.imulink.buffer);

                //Reimposto il contatore del buffer
                IMU.imulink.bytesRecived = 0;

                //Ho ricevuto l'utlimo byte, chiudo il loop
                IMU.data.roll = newValue;

                //Reimposto a 0 ypr
                IMU.imulink.ypr = 0;

                //MAIN_COM.print(IMU.data.yaw); MAIN_COM.print(" "); MAIN_COM.print(IMU.data.pitch); MAIN_COM.print(" "); MAIN_COM.println(IMU.data.roll);

                break;

        }

    }

}

void updateEngines() {

	// int pwm_SE = Radio.data.pwm;
	// int pwm_NW = Radio.data.pwm;
 //    int pwm_NE = Radio.data.pwm;
 //    int pwm_SW = Radio.data.pwm;

    int pwm_NW = Radio.data.pwm + rollPID.output - pitchPID.output;
    int pwm_NE = Radio.data.pwm - rollPID.output - pitchPID.output;
    int pwm_SW = Radio.data.pwm + rollPID.output + pitchPID.output;
    int pwm_SE = Radio.data.pwm - rollPID.output + pitchPID.output;

    if (Radio.data.pwm > INPUT_PWM_MAX || Radio.data.pwm < INPUT_PWM_MIN) {

    	/* Shut down engines */
        M_NW.esc.writeMicroseconds(ESC_MIN_MICROS);
        M_NE.esc.writeMicroseconds(ESC_MIN_MICROS);
        M_SW.esc.writeMicroseconds(ESC_MIN_MICROS);
        M_SE.esc.writeMicroseconds(ESC_MIN_MICROS);

    }
    else {

		/* 	Se mi inclino a sinistra, ho un roll pid output negativo quindi sottraggo per compensare che in realtà va a sommare e ai motori ad ovest sottraggo sommando
			Discorso analogo per il pitch */

        M_NW.esc.writeMicroseconds(pwm_NW);
        M_NE.esc.writeMicroseconds(pwm_NE);
        M_SW.esc.writeMicroseconds(pwm_SW);
        M_SE.esc.writeMicroseconds(pwm_SE);

    }
 }

 void computePID(float realAngle, float desiredAngle, float * output ,float * lastError, float * errorSum, unsigned long * interval, float kP, float kI, float kD) {

    /* Quanto tempo è passato dall'ultima computazione */
   unsigned long now = millis();
   float dt = (float)(now - *(interval));

   /* Calcolo le variabili di Guadagno proporzionale, Integrazione e Derivazione */

   //Fattore proporzionale
   float error = desiredAngle - realAngle;

   //Fattore Integrale : Integro l'errore in dt
   float sum = (error * dt);
   if (abs(*(errorSum) + sum) < 1500) {
        *(errorSum) += sum;
   }
   else {
        *(errorSum) = 1500;
   }


   //Fattore Derivato per prevedere il comportamento "nel futuro"
   float dErr = (error - *(lastError)) / dt;

   /* Calcolo l'output secondo l'equazione  */
   /* Esso rappresenta un offset da sommare o sottrarre */
   *(output) = (kP * error) + (kI * *(errorSum)) + (kD * dErr);

   /* Ricordo alcune variabili per il PID successivo */
   *(lastError) = error;
   *(interval) = now;

 }

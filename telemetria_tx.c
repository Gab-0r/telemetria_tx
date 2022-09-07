#include <stdio.h>
#include "pico/stdlib.h"
#include "nrf24_driver.h"
#include "mpu9250.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include "hardware/timer.h"

#define PI  3.14159265

//Variables de la IMU
int16_t acceleration[3], gyro[3], gyroCal[3], eulerAngles[2], fullAngles[2], magnet[3];
absolute_time_t timeOfLastCheck;

//Variables velocidad del viento
const uint interruptPin = 16;
uint count_holes = 0;
uint count_steps = 0;
const uint holes = 20; //Agujeros del encoder
repeating_timer_t timer;
float radSeg = 0;
float windSpeed = 0;
float radio = 0.08;

//Variables dirección del viento
const int direccionPin = 28;
uint16_t direccionAdc;
float direccionGrados;

//Factor de converion del adc
const float conversion_factor = 3.3f / (1 << 12);

//Funciones sensor de viento
void wind_init(void);
void addHole();
void getSpeed();
bool everySecond();
void setDireccion();

//Funciones de la IMU
void init_mpu9250(int loop);
void updateAngles();
void printDataImu();

char str[100];

//Direcciones de los pipes RF
const uint8_t pipezero_addr[5] = {0x37,0x37,0x37,0x37,0x37};
const uint8_t pipeone_addr[5] = {0xC7,0xC7,0xC7,0xC7,0xC7};
const uint8_t pipetwo_addr[5] = {0xC8,0xC7,0xC7,0xC7,0xC7};

int main()
{
    stdio_init_all();
    sleep_ms(3000);

    wind_init();

    pin_manager_t pins_rf = { 
        .copi = 3,
        .cipo = 4, 
        .sck = 2,
        .csn = 5, 
        .ce = 6 
    };

    nrf_manager_t my_config = {
        // AW_3_BYTES, AW_4_BYTES, AW_5_BYTES
        .address_width = AW_5_BYTES,
        // dynamic payloads: DYNPD_ENABLE, DYNPD_DISABLE
        .dyn_payloads = DYNPD_ENABLE,
        // retransmission delay: ARD_250US, ARD_500US, ARD_750US, ARD_1000US
        .retr_delay = ARD_500US,
        // retransmission count: ARC_NONE...ARC_15RT
        .retr_count = ARC_10RT,
        // data rate: RF_DR_250KBPS, RF_DR_1MBPS, RF_DR_2MBPS
        .data_rate = RF_DR_1MBPS,
        // RF_PWR_NEG_18DBM, RF_PWR_NEG_12DBM, RF_PWR_NEG_6DBM, RF_PWR_0DBM
        .power = RF_PWR_NEG_12DBM,
        // RF Channel 
        .channel = 120,
    };

    // SPI baudrate
    uint32_t my_baudrate = 5000000;

    nrf_client_t my_nrf;

    // initialise my_nrf
    nrf_driver_create_client(&my_nrf);

    // configure GPIO pins and SPI
    my_nrf.configure(&pins_rf, my_baudrate);

    // not using default configuration (my_nrf.initialise(NULL)) 
    my_nrf.initialise(&my_config);


    //set to Standby-I Mode
    my_nrf.standby_mode();

    // payload sent to receiver data pipe 0
    //Estructura de datos para enviar los datos de la IMU
    typedef struct payload_zero_s {
        uint8_t tagAcel;
        int16_t acelX;
        int16_t acelY;
        int16_t acelZ;
        uint8_t tagGyro;
        int16_t gyroX;
        int16_t gyroY;
        int16_t gyroZ;
        uint8_t tagMag;
        int16_t magX;
        int16_t magY;
        int16_t magZ;
    } payload_zero_t;

    // payload sent to receiver data pipe 1
    typedef struct payload_one_s {
        int16_t windDir;
        int16_t windSpeed; 
    } payload_one_t;

    typedef struct payload_two_s { int16_t angle1; int16_t angle2; } payload_two_t;

    // result of packet transmission
    fn_status_t success;

    uint64_t time_sent = 0; // time packet was sent
    uint64_t time_reply = 0; // response time after packet sent

    init_mpu9250(100);

    while(1){
        updateAngles();
        printDataImu();
        setDireccion();

        payload_zero_t payload_zero = {
            .acelX = acceleration[0],
            .acelY = acceleration[1],
            .acelZ = acceleration[2],
            .gyroX = gyro[0] - gyroCal[0],
            .gyroY = gyro[1] - gyroCal[1],
            .gyroZ = gyro[2] - gyroCal[2],
            .magX = magnet[0],
            .magY = magnet[1],
            .magZ = magnet[2]
        };

        payload_one_t payload_one = {
            .windDir = direccionGrados,
            .windSpeed = windSpeed 
           
        };

        // payload sent to receiver data pipe 2
        payload_two_t payload_two = { .angle1= fullAngles[0], .angle2 = fullAngles[1]};

        //send to receiver's DATA_PIPE_0 address
        my_nrf.tx_destination(pipezero_addr);

        // time packet was sent
        time_sent = to_us_since_boot(get_absolute_time()); // time sent

        // send packet to receiver's DATA_PIPE_0 address
        success = my_nrf.send_packet(&payload_zero, sizeof(payload_zero));

        // time auto-acknowledge was received
        time_reply = to_us_since_boot(get_absolute_time()); // response time

        if (success)
        {
        printf("\nPacket sent:- Response: %lluμS | Payload: %d,%d,%d,%d,%d,%d,%d,%d,%d\n",time_reply - time_sent, 
        payload_zero.acelX, payload_zero.acelY, payload_zero.acelZ, payload_zero.gyroX, payload_zero.gyroY, payload_zero.gyroZ,
        payload_zero.magX, payload_zero.magY, payload_zero.magZ);

        } else {

        printf("\nPacket not sent:- Receiver not available.\n");
        }

        sleep_ms(30);

        // send to receiver's DATA_PIPE_1 address
        my_nrf.tx_destination(pipeone_addr);

        // time packet was sent
        time_sent = to_us_since_boot(get_absolute_time()); // time sent

        // send packet to receiver's DATA_PIPE_1 address
        success = my_nrf.send_packet(&payload_one, sizeof(payload_one));
        
        // time auto-acknowledge was received
        time_reply = to_us_since_boot(get_absolute_time()); // response time

        if (success)
        {
        printf("\nPacket sent:- Response: %lluμS | Payload: %d,%d,%d,%d\n", time_reply - time_sent, payload_one.windSpeed, payload_one.windDir);

        } else {

        printf("\nPacket not sent:- Receiver not available.\n");
        }

        sleep_ms(30);

        // send to receiver's DATA_PIPE_2 address
        my_nrf.tx_destination(pipetwo_addr);

        // time packet was sent
        time_sent = to_us_since_boot(get_absolute_time()); // time sent

        // send packet to receiver's DATA_PIPE_2 address
        success = my_nrf.send_packet(&payload_two, sizeof(payload_two));
        
        // time auto-acknowledge was received
        time_reply = to_us_since_boot(get_absolute_time()); // response time

        if (success)
        {
        printf("\nPacket sent:- Response: %lluμS | Payload: %d & %d\n",time_reply - time_sent, payload_two.angle1, payload_two.angle2);

        } else {

        printf("\nPacket not sent:- Receiver not available.\n");
        }

        sleep_ms(30);
    }
}

void init_mpu9250(int loop){
    start_spi();
    calibrate_gyro(gyroCal, loop);
    mpu9250_read_raw_accel(acceleration);
    calculate_angles_from_accel(eulerAngles, acceleration);
    timeOfLastCheck = get_absolute_time();
}

void updateAngles(){
    mpu9250_read_raw_accel(acceleration);
    mpu9250_read_raw_gyro(gyro);
    mpu9250_read_raw_magneto(magnet);
    gyro[0] -= gyroCal[0];
    gyro[1] -= gyroCal[1];
    gyro[2] -= gyroCal[2];
    calculate_angles(eulerAngles, acceleration, gyro, absolute_time_diff_us(timeOfLastCheck, get_absolute_time()));
    timeOfLastCheck = get_absolute_time();
    convert_to_full(eulerAngles, acceleration, fullAngles);
}

void printDataImu(){
    printf("%d,%d,%d\n", acceleration[0], acceleration[1], acceleration[2]); //Acelerometro XYZ
    printf("%d,%d,%d\n", gyro[0] - gyroCal[0], gyro[1] - gyroCal[1], gyro[2] - gyroCal[2]);//Giroscopio
    printf("%d,%d,%d\n", magnet[0], magnet[1], magnet[2]);
    //printf("Euler. Roll = %d, Pitch = %d\n", eulerAngles[0], eulerAngles[1]);
    //printf( "%d,%d\n", fullAngles[0], fullAngles[1]);
}

void wind_init(){
    adc_init();

    //GPIOs
    gpio_init(interruptPin);

    //IRQs
    gpio_set_irq_enabled_with_callback(interruptPin, GPIO_IRQ_EDGE_RISE, true, &addHole);

    //Alarmas
    add_repeating_timer_ms(1000, everySecond, NULL, &timer);

    //Dirección
    adc_gpio_init(direccionPin);
    adc_select_input(2);

    sleep_ms(10);
}

void addHole(){    
    count_holes += 1;
}

bool everySecond(){
    getSpeed();
}

void getSpeed(){
    radSeg = ((count_holes * 60) / holes)*2*PI/60;
    windSpeed = radSeg * radio * 100; //[cm/s]
    printf("Velocidad: %f \n", windSpeed);
    count_holes = 0;
}

void setDireccion(){
    direccionAdc = adc_read();
    // printf("Raw value: 0x%03x, voltage: %f V\n", direccionAdc, direccionAdc * conversion_factor);
    direccionGrados = (direccionAdc * conversion_factor)*(359.0/3.3);
    printf("Direccion: %f °\n", direccionGrados);
}
//////////////////////// Harware Initialization ////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
#include "mbed.h"
#include "rtos.h"
#include <math.h>
#include <ctype.h>

//Photointerrupter input pins
#define I1pin D2
#define I2pin D11
#define I3pin D12

//Incremental encoder input pins
#define CHApin   D7
#define CHBpin   D8  

//Motor Drive output pins   //Mask in output byte
#define L1Lpin D4           //0x01
#define L1Hpin D5           //0x02
#define L2Lpin D3           //0x04
#define L2Hpin D6           //0x08
#define L3Lpin D9           //0x10
#define L3Hpin D10          //0x20


//Mapping from sequential drive states to motor phase outputs
/*
State   L1  L2  L3
0       H   -   L
1       -   H   L
2       L   H   -
3       L   -   H
4       -   L   H
5       H   L   -
6       -   -   -
7       -   -   -
*/
//Drive state to output table
const int8_t driveTable[] = {0x12,0x18,0x09,0x21,0x24,0x06,0x00,0x00};

//Mapping from interrupter inputs to sequential rotor states. 0x00 and 0x07 are not valid
const int8_t stateMap[] = {0x07,0x05,0x03,0x04,0x01,0x00,0x02,0x07};  
//const int8_t stateMap[] = {0x07,0x01,0x03,0x02,0x05,0x00,0x04,0x07}; //Alternative if phase order of input or drive is reversed

//Phase lead to make motor spin
int8_t lead;  //2 for forwards, -2 for backwards
Serial pc(USBTX, USBRX);
//Status LED
DigitalOut led1(LED1);

float target_spd;
float target_rot;
float pwm_duty_cycle = 0.5;
float wait_time;


//Photointerrupter inputs
DigitalIn I1(I1pin);
DigitalIn I2(I2pin);
DigitalIn I3(I3pin);
DigitalIn CHA(CHApin);
DigitalIn CHB(CHBpin);

//Motor Drive outputs
DigitalOut L1H(L1Hpin);
DigitalOut L2H(L2Hpin);
DigitalOut L3H(L3Hpin);

PwmOut* L1Lpwm;
PwmOut* L2Lpwm;
PwmOut* L3Lpwm;

DigitalOut* L1Ldo;
DigitalOut* L2Ldo;
DigitalOut* L3Ldo;

InterruptIn I1State(I1pin);
InterruptIn I2State(I2pin);
InterruptIn I3State(I3pin);

InterruptIn ASlot(CHApin);


/////////////////////////////// State Def //////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////



//Set a given drive state
void motorOut(int8_t driveState){
    
    //Lookup the output byte from the drive state.
    int8_t driveOut = driveTable[driveState & 0x07]; //0 is false, anything else is true
    
    DigitalOut L1L = *L1Ldo;
    DigitalOut L2L = *L2Ldo;
    DigitalOut L3L = *L3Ldo;
      
    //Turn off first
    if (~driveOut & 0x01) L1L = 0;
    if (~driveOut & 0x02) L1H = 1;
    if (~driveOut & 0x04) L2L = 0;
    if (~driveOut & 0x08) L2H = 1;
    if (~driveOut & 0x10) L3L = 0;
    if (~driveOut & 0x20) L3H = 1;
    
    //Then turn on
    if (driveOut & 0x01) L1L = 1;
    if (driveOut & 0x02) L1H = 0;
    if (driveOut & 0x04) L2L = 1;
    if (driveOut & 0x08) L2H = 0;
    if (driveOut & 0x10) L3L = 1;
    if (driveOut & 0x20) L3H = 0;
}


//motorOut equivalent for PWM control
void motorOutPwm(int8_t driveState){
    
    PwmOut L1L = *L1Lpwm;
    PwmOut L2L = *L2Lpwm;
    PwmOut L3L = *L3Lpwm;
    
    //Lookup the output byte from the drive state.
    int8_t driveOut = driveTable[driveState & 0x07]; //0 is false, anything else is true
      
    //Turn off first
    if (~driveOut & 0x01) L1L = 0;
    if (~driveOut & 0x02) L1H = 1;
    if (~driveOut & 0x04) L2L = 0;
    if (~driveOut & 0x08) L2H = 1;
    if (~driveOut & 0x10) L3L = 0;
    if (~driveOut & 0x20) L3H = 1;
    
    //Then turn on
    if (driveOut & 0x01){
         L1L.write(pwm_duty_cycle);
    }
    if (driveOut & 0x02) L1H = 0;
    
    if (driveOut & 0x04){
         L2L.write(pwm_duty_cycle);
    }
    if (driveOut & 0x08) L2H = 0;
    
    if (driveOut & 0x10){
        L3L.write(pwm_duty_cycle);
    }
    if (driveOut & 0x20) L3H = 0;
}


//turn motor off
void MotorOff(){
    DigitalOut L1L = *L1Ldo;
    DigitalOut L2L = *L2Ldo;
    DigitalOut L3L = *L3Ldo;
    
    L1L = 0;
    L1H = 1;
    L2L = 0;
    L2H = 1;
    L3L = 0;
    L3H = 1;
}
    
    
    //Convert photointerrupter inputs to a rotor state
inline int8_t readRotorState(){
    return stateMap[I1 + 2*I2 + 4*I3];
    }

//Basic synchronisation routine    
int8_t motorHome() {
    //Put the motor in drive state 0 and wait for it to stabilise
    motorOut(0);
    wait(1.0);
    
    //Get the rotor state
    return readRotorState();
}

int8_t motorHomePwm() {
    //Put the motor in drive state 0 and wait for it to stabilise
    motorOutPwm(0);
    wait(1.0);
    
    //Get the rotor state
    return readRotorState();
}

////////////////////////////////// Threading ///////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

Thread rotor_thread;

//digital rotor functin
void rotor_thread_func(){
    int8_t orState = 0;
    orState = motorHome();
    int8_t intState = 0;
    int8_t intStateOld = 0;
    while (1) {
        intState = readRotorState();
        if (intState != intStateOld) {
            intStateOld = intState;
            motorOut((intState-orState+lead+6)%6); //+6 to make sure the remainder is positive
        }
        Thread::wait(wait_time);
    }  
}

//pwm rotor function
void rotor_thread_func_pwm(){
    int8_t orState = 0;
    orState = motorHomePwm();
    int8_t intState = 0;
    int8_t intStateOld = 0;
    int motor_update = 0;
    pc.printf("starting rotor thread\n\r");
    while (1) {
        intState = readRotorState();
        if (intState != intStateOld) {
            intStateOld = intState;
            motor_update = (intState-orState+lead+6)%6;
            motorOutPwm(motor_update);
        }
        
        Thread::wait(20);
    }  
}
int countCH = 1;
void count(){
    countCH++;   
}

int countIx = 1;
void countI(){
    countIx++;   
}

//////////////////////////////////// Main //////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

int main() {
    ////////////////// BEGIN INPUT PARSING ///////////////////////
    pc.printf("Enter commands\n\r");
    
    char c;
    float n;
    float rot = 0;
    float spd = 0;
    int state = 0;
    int count_rot = 0;
    int count_spd = 0;
    int dec_count_rot = 1;
    int dec_count_spd = 1;
    bool backward = false;
    bool max_rotation = true;
    bool max_speed = true;
    float rotation = 0;
    float speed = 0;
    
    do{
        if(pc.readable()!=0){
            c = pc.getc();
            pc.printf("%c",c);

            // Determine number of rotations
            if(c == 'R'){
                state = 1;
                max_rotation = false;
            } 
            else if(isdigit(c) && state == 1 && count_rot<=3){
                    n = c -'0';
                    rot = 10*rot + n; 
                    count_rot++;
            } 
            else if (c == '.' && state == 1){
                state = 3;
            }            
            else if(isdigit(c) && state == 3 && dec_count_rot<=2){
                    n = c -'0';
                    rot = rot + n*pow(0.1,dec_count_spd); 
                    dec_count_rot++;
            } 
            // Determine speed
            else if(c == 'V' && !(state == 1 || state == 3)){
                state = 2;
                max_rotation = true;
                max_speed = false;
            }
            else if(c == 'V' && (state == 1 || state == 3)){
                state = 2;
                max_speed = false;
            } 
            else if(isdigit(c) && state == 2 && count_spd<=3){
                    n = c -'0';
                    spd = 10*spd + n; 
                    count_spd++;
            } 
            else if (c == '.' && state == 2){
                state = 4;
            }
            else if(isdigit(c) && state == 4 && dec_count_spd<=2){
                    n = c -'0';
                    spd = spd + n*pow(0.1,dec_count_spd); 
                    dec_count_spd++;
            }
            
            // Determine direction of rotation
            else if (c == '-' && (state == 1 || state == 2)){
                backward = true;
            } 

            // Exit 
            else if(c == '\r'){
                state = 5;
                rotation = rot;
                speed = spd;
            }
            else{
                state = 5;
            }
        }
    } while (state != 5);
    
 ////////////////////////// END INPUT PARSING //////////////////////////////
    pc.printf("\n\r");
    target_spd = speed;
    target_rot = rotation;
    if(!backward) lead = 2;
    else lead = -2;
    
    ASlot.rise(&count);

    if(max_speed && max_rotation){ //max speed, no rotation limit, just turn 
        target_spd = 100; //max speed
        wait_time = 0;
        rotor_thread.start(rotor_thread_func);
    }
    else if(max_speed){ //no speed limit, rotation limit
        pc.printf("max speed and limited distance\n\r");
        PwmOut L1L(L1Lpin);
        PwmOut L2L(L2Lpin);
        PwmOut L3L(L3Lpin);
        L1L.period(0.1f);
        L2L.period(0.1f);
        L3L.period(0.1f);
        L1Lpwm = &L1L;
        L2Lpwm = &L2L;
        L3Lpwm = &L3L;
        rotor_thread.start(rotor_thread_func_pwm); 

        float kp_dist = 0.06;
        float ki_dist = 0.003;
        float distance;
        float speed;
        float acc = 0;
        float cur_time;
        
        Timer time;
        time.start();
       
        while(1){
            if(countCH % 19 == 0){
               cur_time = time.read();
               speed = 1/(6*cur_time);
               time.reset();
               distance = countCH/117.0;
               pc.printf("distance: %f, duty cycle: %f, speed: %f\n\r",distance,pwm_duty_cycle,speed);
               
               //acc += ki_dist * (target_rot - distance) *  cur_time;
               pwm_duty_cycle = kp_dist * (target_rot - distance); //+ acc;
               if (pwm_duty_cycle > 1) pwm_duty_cycle = 1;
               if (pwm_duty_cycle < 0) pwm_duty_cycle = 0;
            }
        }
    }
    else if(max_rotation){ //speed limit, no rotation limit
        pc.printf("max distance and limited speed\n\r");
        pc.printf("desired speed: %f\n\r",target_spd);
        PwmOut L1L(L1Lpin);
        PwmOut L2L(L2Lpin);
        
        PwmOut L3L(L3Lpin);
        L1L.period(0.1f);
        L2L.period(0.1f);
        L3L.period(0.1f);
        L1Lpwm = &L1L;
        L2Lpwm = &L2L;
        L3Lpwm = &L3L;
        rotor_thread.start(rotor_thread_func_pwm);
        
            
        float kp_speed = 0.1;
        float ki_speed = 0.001;
        float speed;
        float acc = 0;
        float cur_time;
        
        Timer time;
        time.start();
       
        while(1){
            if(countCH % 19 == 0){
               cur_time = time.read();
               speed = 1/(6*cur_time);
               time.reset();
               
               pc.printf("speed: %f, pwm_duty_cycle: %f\n\r",speed,pwm_duty_cycle);
               
               acc += ki_speed * (target_spd - speed) * cur_time;
               pwm_duty_cycle =  kp_speed * (target_spd - speed) + acc;
              
               if (pwm_duty_cycle > 1) pwm_duty_cycle = 1;
               if (pwm_duty_cycle < 0) pwm_duty_cycle = 0;
            }
        }
    } 
    else{ //rotation and speed limit
        pc.printf("limited distance and limited speed\n\r");
        PwmOut L1L(L1Lpin);
        PwmOut L2L(L2Lpin);
        PwmOut L3L(L3Lpin);
        L1L.period(0.1f);
        L2L.period(0.1f);
        L3L.period(0.1f);
        L1Lpwm = &L1L;
        L2Lpwm = &L2L;
        L3Lpwm = &L3L;
        rotor_thread.start(rotor_thread_func_pwm); 

        float kp_dist = 0.01;
        float ki_dist = 0.003;
        float distance;

        float acc_speed = 0;
        float acc_dist = 0;
        float cur_time;
        
        float kp_speed = 0.01;
        float ki_speed = 0.001;
        float speed;
        
        float pwm_dist;
        float pwm_spd;

        
        Timer time;
        time.start();
       
        while(1){
            if(countCH % 19 == 0){
               cur_time = time.read();
               time.reset();
               distance = countCH/117.0;
               speed = 1/(6*cur_time);
               //pc.printf("distance: %f\n\r",distance);
               
               acc_dist += ki_dist * (target_rot - distance) *  cur_time;
               pwm_dist = kp_dist * (target_rot - distance) + acc_dist;
               
               //acc_dist += ki_speed * (target_spd - speed) * cur_time;
               pwm_spd =  kp_speed * (target_spd - speed);// + acc_dist;
               
               if(pwm_dist < pwm_spd){
                   pwm_duty_cycle = pwm_dist;
               }
               else{
                    pwm_duty_cycle = pwm_spd;
                }
               
               if (pwm_duty_cycle > 1) pwm_duty_cycle = 1;
               if (pwm_duty_cycle < 0) pwm_duty_cycle = 0;
            }
        }
    }
}


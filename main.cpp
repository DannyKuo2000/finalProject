#include "mbed.h"
#include "bbcar.h"
#include <iostream>
#include <string.h>
#define CENTER_BASE 1500
#define unitsFC 360                          // Units in a full circle
#define dutyScale 1000                       // Scale duty cycle to 1/000ths
#define dcMin 29                             // Minimum duty cycle
#define dcMax 971                            // Maximum duty cycle
#define q2min unitsFC/4                      // For checking if in 1st uadrant
#define q3max q2min * 3                      // For checking if in 4th uadrant

Ticker servo_ticker;
Ticker servo_feedback_ticker;

Ticker feedback_ticker;
Ticker feedback_ticker2;
Ticker control_ticker;

PwmIn servo0_f(D9), servo1_f(D10); // servo
PwmOut servo0_c(D11), servo1_c(D12);
BBCar car(servo0_c, servo0_f, servo1_c, servo1_f, servo_ticker, servo_feedback_ticker);

BufferedSerial serdev(D1, D0, 9600); // XBee

BusInOut qti_pin(D4, D5, D6, D7);
parallax_qti qti1(qti_pin); // QTI

DigitalInOut pin8(D8);
parallax_laserping  ping1(pin8); // Laser

DigitalOut myled(LED1); // LED

volatile int angle, targetAngle = 3;              // Global shared ariables
volatile int Kp = 1;                          // Proportional constant
volatile float tCycle;
volatile int theta;
volatile int thetaP;
volatile int turns = 0;

volatile int angle2, targetAngle2 = 3;              // Global shared ariables
volatile int Kp2 = 1;                          // Proportional constant
volatile float tCycle2;
volatile int theta2;
volatile int thetaP2;
volatile int turns2 = 0;

double s = 4; // speed variable
int scanM[10]; // for scan

void scan();
void rotateFindBlack();
void rotateFindBlackR();

void feedback360L(){                           // Position monitoring
    tCycle = servo0_f.period();
    int dc = dutyScale * servo0_f.dutycycle();
    theta = (unitsFC - 1) -                   // Calculate angle
            ((dc - dcMin) * unitsFC)
            / (dcMax - dcMin + 1);
    if(theta < 0)                             // Keep theta valid
        theta = 0;
    else if(theta > (unitsFC - 1))
        theta = unitsFC - 1;

    // If transition from quadrant 4 to
    // quadrant 1, increase turns count.
    if((theta < q2min) && (thetaP > q3max))
        turns++;
    // If transition from quadrant 1 to
    // quadrant 4, decrease turns count.
    else if((thetaP < q2min) && (theta > q3max))
        turns --;

     // Construct the angle measurement from the turns count and
     // current theta value.
    if(turns >= 0)
        angle = (turns * unitsFC) + theta;
    else if(turns <  0)
        angle = ((turns + 1) * unitsFC) - (unitsFC - theta);

    thetaP = theta;                           // Theta previous for next rep
}

void feedback360R(){                           // Position monitoring
    tCycle2 = servo1_f.period();
    int dc2 = dutyScale * servo1_f.dutycycle();
    theta2 = (unitsFC - 1) -                   // Calculate angle
            ((dc2 - dcMin) * unitsFC)
            / (dcMax - dcMin + 1);
    if(theta2 < 0)                             // Keep theta valid
        theta2 = 0;
    else if(theta2 > (unitsFC - 1))
        theta2 = unitsFC - 1;

    // If transition from quadrant 4 to
    // quadrant 1, increase turns count.
    if((theta2 < q2min) && (thetaP2 > q3max))
        turns2++;
    // If transition from quadrant 1 to
    // quadrant 4, decrease turns count.
    else if((thetaP2 < q2min) && (theta2 > q3max))
        turns2 --;

     // Construct the angle measurement from the turns count and
     // current theta value.
    if(turns2 >= 0)
        angle2 = (turns2 * unitsFC) + theta2;
    else if(turns2 <  0)
        angle2 = ((turns2 + 1) * unitsFC) - (unitsFC - theta2);

    thetaP2 = theta2;                           // Theta previous for next rep
}

void turnLeft90(int i) {
    printf("Now turn left 90!\n");
    car.stop();
    ThisThread::sleep_for(500ms);

    if (i == 1) {
        printf("forward\n");
        car.goStraight(25);
        ThisThread::sleep_for(2000ms);
    }
    
    car.stop();
    ThisThread::sleep_for(500ms);
    printf("turn\n");
    car.turn(-28, 27);
    ThisThread::sleep_for(4s);

    printf("stop\n");
    car.stop();
    ThisThread::sleep_for(3000ms);

}

void turnRight90() {
    printf("Now turn right 90!\n");
    car.stop();
    ThisThread::sleep_for(500ms);
    printf("forward\n");
    car.goStraight(25);
    ThisThread::sleep_for(2000ms);
    car.stop();
    ThisThread::sleep_for(500ms);
    printf("turn\n");
    car.turn(45, -50); 
    ThisThread::sleep_for(1600ms);
    //ThisThread::sleep_for(5200ms);
    printf("stop\n");
    car.stop();
    ThisThread::sleep_for(3000ms);
}

void turn180() {
    printf("Now turn left 180!\n");
    car.turn(-35, -35);
    ThisThread::sleep_for(4500ms);

    car.turn(-50, 34);
    ThisThread::sleep_for(4500ms);
    car.stop();
    ThisThread::sleep_for(500ms);
}

void rotateFindBlack()
{
    int pattern;

    printf("Find black!\n");
    car.stop();
    ThisThread::sleep_for(300ms);
    car.turn(-50, 34);
    while (true) {
        pattern = (int)qti1;
        if (pattern == 0b1000) {
            break;
        }
        ThisThread::sleep_for(5ms);
    }
    car.stop();
    ThisThread::sleep_for(500ms);

    
}

void rotateFindBlackR()
{
    int pattern;

    printf("In rotateFindBlackR, Find black!\n");
    serdev.write("In rotateFindBlackR, Find black!\n", 33);
    car.stop();
    ThisThread::sleep_for(300ms);
    car.turn(45, -50);
    while (true) {
        pattern = (int)qti1;
        if (pattern == 0b0001) {
            break;
        }
        ThisThread::sleep_for(5ms);
    }
    car.stop();
    ThisThread::sleep_for(500ms);

    
}

void mad()
{
    for (int i = 0; i < 3; i++) {
        printf("Being mad!\n");
        serdev.write("Being mad!\n", 11);
        car.goStraight(200); // 45:25 auto+20
        ThisThread::sleep_for(300ms);
        car.turn(-20, -42);
        ThisThread::sleep_for(2000ms);
        car.stop();
        ThisThread::sleep_for(500ms);
    }
    
}

void happy()
{
    printf("Being happy\n");
    serdev.write("Being happy!\n", 13);
    car.stop();
    for (int i = 0; i < 10; i++) {
        myled = 1;          // set LED1 pin to high
        ThisThread::sleep_for(100ms);
        myled.write(0);     // set LED1 pin to low
        ThisThread::sleep_for(100ms);
    }
    
}


void scan() {
    float distance;
    int distBufferSize = 200;
    float distBuffer[distBufferSize]; // store distance data
    for (int i = 0; i < 10; i++) { // relevant with label2 
        distBuffer[i] = 18; // for prevent fault, need to between upper and lower
    }
    
    
    //float boundaryBuffer[4] = {1000, 0, 0, 0}; // for prevent fault
    int recordSize = 50;
    int record[recordSize];

    for (int i = 0; i < recordSize; i++) {
        record[i] = -1;
    }
    int recordEdge[recordSize];
    for (int i = 0; i < recordSize; i++) {
        recordEdge[i] = distBufferSize + 1; // for cutting the tail
    }
    
    int recordLabel = 0;

    // to determint the 0 or 1
    int upper = 20;
    int lower = 15;
    
    //printf("Hi!\n");
    car.turn(40, -40); // 5sec <=> 115 degree

    int finish = 0;
    while (finish == 0) {
        distance = (float)ping1;

        int zero_cd_label;
        int one_cd_label;

        if (distance < 15) { // < 20
            printf("Found boundary\n");
            int label2 = 10;

            while (true) { // scan
                distance = (float)ping1; 
                printf("%f %d\n", distance, label2);
                distBuffer[label2] = distance;
                
                // checking
                int k_one = 1; // checking range for slit
                int write_1 = 0;
                //float sum2 = 0;
                for (int i = label2; i > label2 - k_one; i--) {
                    if (distBuffer[i] <= upper) {
                        write_1 = 0;
                        break;
                    }
                    else {
                        write_1 = 1;
                        //continue;
                    }
                }

                int k_zero = 6; // checking range
                int write_0 = 0;
                for (int i = label2; i > label2 - k_zero; i--) {
                    if (distBuffer[i] >= lower) {
                        write_0 = 0;
                        break;
                    }
                    else {
                        write_0 = 1;
                        //continue;
                    }
                    
                }
                
                int stop = 0;
                for (int i = label2; i > label2 - 6; i--) { // how many 0 to stop
                    if (distBuffer[i] <= upper) {
                        stop = 0;
                        break;
                    }
                    else {
                        stop = 1;
                        //continue;
                    }
                    
                }

                if (write_0 == 1 && label2 - zero_cd_label >= 6) { // control CD
                    zero_cd_label = label2;
                    record[recordLabel] = 0;
                    recordEdge[recordLabel] = label2;
                    recordLabel += 1;
                    printf("Find 0\n");
                }
                else if (write_1 == 1 && label2 - one_cd_label >= 2) { // control CD
                    one_cd_label = label2;
                    record[recordLabel] = 1;
                    recordEdge[recordLabel] = label2;
                    recordLabel += 1;
                    printf("Find 1\n");
                }
                else if (stop == 1){
                    printf("Find another boundary!\n");
                    rotateFindBlackR();
                    break;
                }

                if (label2 == distBufferSize - 1)
                    label2 = 0;
                else
                    label2 += 1;

                ThisThread::sleep_for(100ms);

            }
            // display pattern
            int j = 0;
            for (int i = 0; i < recordSize; i++) {
                if (recordEdge[i] < label2 - 6) {
                    printf("%d ", record[i]);
                    scanM[j] = record[i];
                    j += 1;
                }
                    
            }
            printf("\n");

            finish = 1;
        }
    }
}

int stringCompare(char instr[])
{
    if (instr[0] == 's' && instr[1] == 't' && instr[2] == 'a' && instr[3] == 'r' && instr[4] == 't') {
        return 1;
    }
    else 
        return 0;
}

/*void ptoc(int &pattern, int &lastpattern)
{
    if (lastpattern == -1) { // QTI display
        lastpattern = pattern;
    }
    else if (lastpattern != pattern) {
        switch (pattern) { 
            case 0b1000: 
                char res[5] = {'1', '0', '0', '0', '\0'};
                serdev.write(res, strlen(res));
                break;
            case 0b1100: 
                char res1[5] = {'1', '1', '0', '0', '\0'};
                serdev.write(res, strlen(res));
                break;
            case 0b0100: 
                char res2[5] = {'0', '1', '0', '0', '\0'};
                serdev.write(res, strlen(res));
                break;
            case 0b0110: 
                char res3[5] = {'0', '1', '1', '0', '\0'};
                serdev.write(res, strlen(res));
                break;
            case 0b0010: 
                char res4[5] = {'0', '0', '1', '0', '\0'};
                serdev.write(res, strlen(res));
                break; 
            case 0b0011: 
                char res5[5] = {'0', '0', '1', '1', '\0'};
                serdev.write(res, strlen(res));
                break;
            case 0b0001: 
                char res6[5] = {'0', '0', '0', '1', '\0'};
                serdev.write(res, strlen(res));
                break;
            case 0b0000:
                char res7[5] = {'0', '0', '0', '0', '\0'};
                serdev.write(res, strlen(res));
                break;
            case 0b1001:
                char res8[5] = {'1', '0', '0', '1', '\0'};
                serdev.write(res, strlen(res));
                break;
            case 0b1110:
                char res9[5] = {'1', '1', '1', '0', '\0'};
                serdev.write(res, strlen(res));
                break;
            case 0b0111: 
                char res10[5] = {'0', '1', '1', '1', '\0'};
                serdev.write(res, strlen(res));
                break;
            case 0b1111: 
                char res11[5] = {'1', '1', '1', '1', '\0'};
                serdev.write(res, strlen(res));
                break;
            default: 
                char res12[8] = {'d', 'e', 'f', 'a', 'u', 'l', 't', '\0'};
                serdev.write(res, strlen(res));
                break;
        }
    }
}*/

int main() {
    
    int pattern = -1;
    int lastpattern;
    int start = 0;
    int finish = 0;
    int label = 0;
    int record;
    int counter = 0;
    int nextLR = 0; // for turn left or right
    int nextLR_counter = 0;
    int counter1111 = 0;

    int lastTurn = -1; // recording last turn
    float distance; // for laser

    printf("Hi!\n");
    // waiting for command to start
    char c;
    char instr[32];
    memset(instr, '\0', 32); //clear buffer
    int i = 0;
    serdev.write("Hi, enter 'start' to start!\n", 28);
    while (true) {
        serdev.read(&c, 1);
        if (c != '\r' && c != '\n') {
            printf("%c", c);
            instr[i] = c;
            i++;
        }
        else { //Either '\r' or '\n'
            printf("\n");
            if (stringCompare(instr)) {
                printf("Let's start\n");
                serdev.write("BBCar received: ", 16);
                serdev.write(instr, strlen(instr));
                serdev.write("\n", 1);
                serdev.write("BBCar replied: Start!\n", 22); // +15
                memset(instr, '\0', 32);
                i = 0;
                break;
            }
            else {
                serdev.write("BBCar received: ", 16);
                serdev.write(instr, strlen(instr));
                serdev.write("\n", 1);
                serdev.write("BBCar replied: Invalid command!\n", 32);
                i = 0;
            }
            memset(instr, '\0', 32);
        }
    }

    // start running
    car.goStraight(25);
    while(finish == 0) {
        pattern = (int)qti1;
        label += 1;
        if (start == 0) {
            record = label;
        }

        if (lastpattern == -1) { // QTI display
            lastpattern = pattern;
        }
        else if (lastpattern != pattern) {
            printf("%d\n", pattern);
            //serdev.write(, strlen(pattern)); // need to modify
        }
        
        // servo0(left): 44, -15 ------ 17
        // servo1(right): 24, -32 ------ -4
        switch (pattern) { 
            // toward left
            case 0b1000: 
                car.turn(-42 - s, 27 + s); // 70:50->45:50
                //ThisThread::sleep_for(3ms);
                break; 
            case 0b1100: 
                car.turn(-15 - s, 29 + s); // 70:50->45:50
                //ThisThread::sleep_for(1ms);
                break;
            case 0b0100: 
                car.turn(17 + s/2, 33 + s); // 60:40->45:40
                //ThisThread::sleep_for(1ms);
                break;
            // go straight
            case 0b0110: 
                car.goStraight(21 + s/2); // 45:25 auto+20
                //ThisThread::sleep_for(1ms);
                break;
            // toward right
            case 0b0010: 
                car.turn(43 + s, -4 + s/2); 
                //ThisThread::sleep_for(1ms);
                break; 
            case 0b0011: 
                car.turn(43 + s, -32 - s); 
                //ThisThread::sleep_for(1ms);
                break;
            case 0b0001: 
                car.turn(45 + s, -59 - s); 
                //ThisThread::sleep_for(3ms);
                break;
            case 0b0000: // back
                car.turn(-23 - s, -34 - s); 
                //ThisThread::sleep_for(3ms);
                break;
            case 0b1001: // special
                car.turn(45 + s, -59 - s); 
                //ThisThread::sleep_for(1ms);
                break;

            // preview turn left or right
            case 0b1110:
                if (lastpattern == 0b1110) 
                    nextLR_counter += 1;
                else {
                    //printf("Length: %d\n", nextLR_counter);
                    nextLR_counter = 0;
                }
                if (nextLR_counter > 120) {
                    if (nextLR != -1) {
                        serdev.write("Next turn left!\n", 16);
                        printf("Next turn left!\n");
                    }
                    nextLR = -1;
                }
                //car.goStraight(21); // 45:25 auto+20
                //ThisThread::sleep_for(1ms);
                /*car.turn(-23 - s/2, 16 + s); // 60:40->45:40
                ThisThread::sleep_for(1ms);*/
                break;
            case 0b0111: 
                if (lastpattern == 0b0111) 
                    nextLR_counter += 1;
                else {
                    //printf("Length: %d\n", nextLR_counter);
                    nextLR_counter = 0;
                }
                if (nextLR_counter > 120) {

                    if (nextLR != 1) {
                        printf("Next turn right!\n");
                        serdev.write("Next turn right!\n", 17);
                    }
                    nextLR = 1;
                }
                //car.goStraight(21); // 45:25 auto+20
                //ThisThread::sleep_for(1ms);
                /*car.turn(35 + s, -45 - s/2); 
                ThisThread::sleep_for(1ms);*/
                break;
            // turn left, right or stop______________________________
            case 0b1111: 
                if (lastpattern == 0b1111) 
                    counter1111 += 1;
                else 
                    counter1111 = 0;
                if (nextLR == -1 && counter1111 > 70) {
                    printf("Now turn left!\n");
                    serdev.write("Now turn left!\n", 15);
                    while (true) {
                        pattern = (int)qti1;
                        //printf("%d\n", pattern);
                        car.turn(17, 27);
                        //ThisThread::sleep_for(1ms);
                        if (pattern == 0b1100 || pattern == 0b1000) {
                            nextLR = 0;
                            break;
                        }
                    }
                } 
                else if (nextLR == 1  && counter1111 > 70) {
                    printf("Now turn right!\n");
                    serdev.write("Now turn right!\n", 16);
                    //car.stop();
                    //ThisThread::sleep_for(1s);
                    while (true) {
                        pattern = (int)qti1;
                        //printf("%d\n", pattern);
                        car.turn(45, -4);
                        //ThisThread::sleep_for(1ms);
                        if (pattern == 0b0011 || pattern == 0b0001) {
                            nextLR = 0;
                            break;
                        }
                    }
                }
                else {
                    car.goStraight(21 + s);
                    //ThisThread::sleep_for(1ms);
                }

                if (counter1111 > 200 && start == 1) { // stop or not
                    float length;
                    car.stop();
                    /*printf("turns = %d, angle = %d\n", turns, angle);
                    printf("turns2 = %d, angle2 = %d\n", -turns2, -angle2);
                    length = ((float)angle + (-1)*(float)angle2) / 2 / 360 * 3.14 * 6.5;
                    printf("Length = %f\n", length);*/
                    finish = 1;
                }
                break;
            default: 
                car.goStraight(21 + s); 
                //ThisThread::sleep_for(1ms);
                break;
        }

        // start counting distance
        /*if (counter1111 > 15 && pattern != 15 && start == 0) {
            start = 1;
            counter1111 = 0;
            //printf("start = 1\n");
            feedback_ticker.attach(&feedback360L, 5ms);
            feedback_ticker2.attach(&feedback360R, 5ms);
        }*/
        
        lastpattern = pattern;

        // for Laser____________________________________________________
        distance = (float)ping1;
        //printf("Distance: %f\n", distance);
        if (distance < 7) {
            printf("Counter obstacle, ready to scan!\n");
            serdev.write("Counter obstacle, ready to scan!\n", 33);
            turnLeft90(0);
            for (int i = 0; i < 10; i++) {
                scanM[i] = -1;
            }
            printf("Now scan!\n");
            serdev.write("Now scan!\n", 10);
            scan();
            car.stop();
            ThisThread::sleep_for(500ms);
            printf("Display scan message!\n");
            serdev.write("Display scan message:", 21);
            char scanMC[10]; // stand for scanM char
            for (int i = 0; i < 10; i++) {
                if (scanM[i] == 0)
                    scanMC[i] = '0';
                else if (scanM[i] == 1)
                    scanMC[i] = '1';
                else {
                    break;    
                }
                printf("%d ", scanM[i]);
            }
            printf("\n");
            serdev.write(scanMC, strlen(scanMC));
            serdev.write("\n", 1);

            // examination
            int allzero = 0;
            for (int i = 0; i < 10; i++) {
                if (scanM[i] == 0) {
                    allzero = 1;
                }
                else if (scanM[i] == -1) {
                    break;
                }
                else {
                    allzero = 0;
                    break;
                }
                    
            }
            if (allzero == 1) { // pure obstacle
                printf("There is a pure obstacle\n");
                serdev.write("There is a pure obstacle\n", 25);
            }
            else { // 110010
                if (scanM[0] == 1 && scanM[1] == 1 && scanM[2] == 0 && scanM[3] == 0 && scanM[4] == 1 && scanM[5] == 0) {
                    happy(); // blink
                } // 0000011
                else if (scanM[0] == 0 && scanM[1] == 0 && scanM[2] == 0 && scanM[3] == 0 && scanM[4] == 0 && scanM[5] == 0 && scanM[6] == 1) {
                    mad(); // charge
                }
                else {
                    printf("Can't decode the message!\n");
                    serdev.write("Can't decode the message!\n", 26);
                }
            }

            //pwmout_read_pulsewidth_us(PwmIn servo0_f);
        }
    // finish = 1

        
        
    }
}
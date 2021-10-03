#include "../include/teleCmd.h"

teleCmd::teleCmd(){
    vx = 0;
    vy = 0;
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig){
    tcsetattr(kfd, TCSANOW, &cooked);
    ros::shutdown();
    exit(0);
}

void teleCmd::keyLoop(){
    // get console in raw mode
    char c;
    bool dirty=false;
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked,  sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    // setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOL] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Use arrow keys to move robot");

    for(;;){
        // get the next event from the keyboard  
        if(read(kfd, &c, 1) < 0)
        {
            perror("read():");
            exit(-1);
        }
 
        ROS_DEBUG("value: 0x%02X\n", c);

        switch(c){

            case KEYCODE_UP:
                if(vx >minVal & vx <maxVal) {vx += 0.1;}
                else{vx = vx;}
                dirty = true;
            break;

            case KEYCODE_DOWN:
                if(vx >minVal & vx <maxVal) {vx -= 0.1;}
                else{vx = vx;}
            break;

            case KEYCODE_RIGHT:
                if(vy >minVal & vy <maxVal) {vy += 0.1;}
                else{vy = vy;}
            break;

            case KEYCODE_LEFT:
                if(vy >minVal & vy <maxVal) {vy -= 0.1;}
                else{vy = vy;}
            break;
        }
    }
    return;
}

double teleCmd::getVxCommand(){
    return vx;
}

double teleCmd::getYyCommand(){
    return vy;
}

void teleCmd::setZero(){
    vx = 0;
    vy = 0;
}
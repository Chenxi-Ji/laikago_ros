/*!
 * @file teleCmd.h
 * @brief read keyboard input and send to desired command.h
 * 
 * output number between -1-1 for forwardspeed, sidespeed (in body frame) (for now)
 * will add command for rotation
 */

#include <ros/ros.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>

#ifndef PROJCET_TELECMD_H
#define PROJECT_TELECMD_H

#define KEYCODE_UP    0x41
#define KEYCODE_DOWN  0x42
#define KEYCODE_LEFT  0x44
#define KEYCODE_RIGHT 0x43

class teleCmd{
  public:
    teleCmd();
    void keyLoop();
    double getVxCommand();
    double getYyCommand();
    void setZero();
    // rotation speed cmd for later
  private:
    double vx;
    double vy;

    double maxVal = 1;
    double minVal = -1;
};

#endif
 
/*
 * Click nbfs://nbhost/SystemFileSystem/Templates/Licenses/license-default.txt to change this license
 * Click nbfs://nbhost/SystemFileSystem/Templates/cppFiles/file.h to edit this template
 */

/* 
 * File:   uvc_opencv_example.hpp
 * Author: sygorra
 *
 * Created on 5 novembre 2025, 09:27
 */

#ifndef UVC_OPENCV_EXAMPLE_HPP
#define UVC_OPENCV_EXAMPLE_HPP
#define NO_XWINDOW 1
#define USE_XWINDOW 2
typedef struct{
  int mode_xwindows; 
#define VERBOSE_NONE 0
#define VERBOSE_TIME_MSK 1  
#define VERBOSE_COUNT_MSK 2  
#define VERBOSE_CONV_MSK 4  
  
  int verbose; // <=0 => no message
} struct_opencv_example;


#endif /* UVC_OPENCV_EXAMPLE_HPP */


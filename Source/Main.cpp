/*
 * Main.cpp
 *
 *  Created on: 2016年8月17日
 *      Author: seeing
 */
#include <iostream>
#include "Application/ApplicationManager.h"
#include "Global.h"

int
main (int argc, char* argv[])
{
  ApplicationManager manager;
  
  if (!manager.initialize ())
  {
    return -1;
  }
  
  if (!manager.run ())
  {
    return -1;
  }
  
  manager.pending ();
  
  return 0;
}


float duty;     //variable to hold the duty cycle value (0 - 255)
int count = 0; //variable to count number of 'turns' in challenge 4
void setup() 
{
  DDRC &= B11110000; //set pins 0-3 (sel0, sel1, sel2, sel3) as inputs for rotary selector switch // &= ensures pins 0-3 = 0, whilst maintaining any previously set values for pins 4-7
  DDRA |= B11111111; //set all pins as outputs // initialise proximity LEDs
  DDRJ |= B00001111; //set pins 0-3 as outputs // initialise ground LEDs // |= ensures pins 0-3 = 1, whilst maintaining any previously set values for pins 4-7

    Serial.begin(2000000);  //baud rate set perhaps too fast but this setting improved the performance of the challenge
//    Serial.begin(300);
}

void loop() 
{
  if(selectorPosition() == 0)
  {
    duty = 20;
    lineMovement(); // challenge 1
  }
  else if(selectorPosition() == 1) 
  {
    duty = 21;
    lineMovement(); // challenge 2
  }
  else if(selectorPosition() == 2) 
  {
    duty = 20;
    challenge3();
    // challenge 3
  }
  else if(selectorPosition() == 3) 
  {
    duty = 18;
    challenge4Left();
  } 
  else if(selectorPosition() == 4) 
  {
    duty = 18;
    challenge4Right();
  } 
  else if(selectorPosition() == 5) 
  {
    duty = 18;
    challenge4LeftFast();
  } 
  else if(selectorPosition() == 6) 
  {
    duty = 18;
    challenge4RightFast();
  } 
  else 
  {
    Stop();
    readGroundSensors();
  }
}

unsigned char selectorPosition() //read rotary selector position
{
  return PINC & 0x0F; //read pins 0-3 at port C and return value
}

//CHALLENGE FUNCTIONS 

void lineMovement() // movement algorithm for challange 1 and challenge 2
{
  if(!readGroundSensor(1)&&readGroundSensor(2))  //if front left sensor reads black and front right reads white
    {
      turnLeft(duty);
    }
  else if(readGroundSensor(1)&&!readGroundSensor(2))  //if front left sensor reads white and front right reads black
    {
      turnRight(duty);
    }
  else if(!readGroundSensor(3)&&readGroundSensor(0))  //if back left senso reads white and back right reads black //constrained with && because of part of track where 0 and 3 read black
    {
      turnRightFast(duty);                            //sharp right turn
    }
  else if(!readGroundSensor(0)&&readGroundSensor(3))  //if back left sensor reads blcak and back right reads white //constrained with && because of part of track where 0 and 3 read black
    {
      turnLeftFast(duty);                             //sharp left turn
    }
  else
    {
      forwards(duty);  
    }
}

void challenge3()
{
    int rDelay = random(0,400); //randomise the time for a turn to vary turn angle
    if(!readGroundSensor(1))    
    {
      spinClockwise(duty-1);    //rotate clockwise for a random time between 0-400ms
      delay(rDelay);
    }
    else if(!readGroundSensor(2))
    {
      spinAntiClockwise(duty-1);//rotate anti-clockwise for a random time between 0-400ms
      delay(rDelay);
    }
    else if(!readGroundSensor(1)&&!readGroundSensor(2))
    {
      backwards(duty);
      delay(500);
      int LR = random(0,1);     //randomiser to deterimine direction of rotation
      if(LR = 0)
      {
        spinClockwise(duty);
        delay(rDelay);
      }
      else
      {
        spinAntiClockwise(duty);
        delay(rDelay);
      }
    }
    else if(readProximitySensor(0) < 830) //tuned value for the front sensor - close enough to allow for movement throughout the box, far enough to avoid contact with block
    {
      backwards(duty);
      delay(400);
      int LR = random(0,1);
      if(LR = 0)
      {
        spinClockwise(duty);
        delay(350);
      }
      else
      {
        spinAntiClockwise(duty);
        delay(350);
      }
    }
    else if(readProximitySensor(1) < 810) //tuned value for the front right sensor - close enough to allow for movement throughout the box, far enough to avoid contact with block
    {
      spinAntiClockwise(duty);
      delay(350); 
    }

    else if(readProximitySensor(7) < 800) //tuned value for the front left sensor - close enough to allow for movement throughout the box, far enough to avoid contact with block
    {
      spinClockwise(duty);
      delay(350); 
    }
    else if(readProximitySensor(2) < 855) //tuned value for the right sensor - close enough to allow for movement throughout the box, far enough to avoid contact with block
    {
      turnLeft(duty);
    }
    
    else if(readProximitySensor(6) < 850) ////tuned value for the left sensor - close enough to allow for movement throughout the box, far enough to avoid contact with block
    {
      turnRight(duty);
    }
    else
    {
      forwards(duty);
    }
}

void challenge4Left()
{
  challenge4GridLeft();
  wallFollowingLeft;
}

void challenge4Right()
{
  challenge4GridRight();
  wallFollowingRight;
}

void challenge4LeftFast()
{
  challenge4GridLeftFast();
  wallFollowingLeft;
}

void challenge4RightFast()
{
  challenge4GridRightFast();
  wallFollowingRight;
}

void challenge4GridLeft()
{
                        
  if(count == 0)                    //count is initialised to zero
  {
    forwards(duty);                     //release from the starting square //facing left
    delay(1000);                      //delay to ensure robot is beyond the junction - back sensors no longer read black
    count++;                        ////increase turn counter by 1 //repeats after each turn
  }
  else if(count == 1 && !readGroundSensor(3) && readGroundSensor(0))  //when first corner is approached turn right
  {
    Stop();
    delay(500);
    while(readGroundSensor(0))                                       //while back left sensor reads white to ensure full turn is completed
    {
      spinClockwise(duty);
      forwards(duty);
      delay(500);                                                    //delay to ensure robot is beyond the junction - back sensors no longer read black
      count++;                                                           
    }
  }
  else if(count == 2 && !readGroundSensor(1) && !readGroundSensor(2))  //when second junction (count == 2) is approached go straight ahead
  {
    forwards(duty);
    delay(500);                                                        //delay to ensure robot is beyond the junction - back sensors no longer read black
    count++;
  }
  else if(count == 3 && !readGroundSensor(3) && !readGroundSensor(0)) //when third junction (count == 3) is approached turn left
  {
    Stop();
    delay(500);                                                        
    if(!readGroundSensor(0)&&readGroundSensor(3))                                       //spin until back left reads black and back right reads white simultaneously
    {
      Stop();
      delay(500);
      forwards(duty);
      delay(500);                                                             //delay to ensure robot is beyond the junction - back sensors no longer read black
      count++;
    }
    else
    {
      spinAntiClockwise(duty);
    }
  }
  else if(count == 4 && !readGroundSensor(3) && !readGroundSensor(0)) //when forth junction (count == 4) is approached turn right
  {
    Stop();
    delay(500);
    if(readGroundSensor(0)&&!readGroundSensor(3))                                       //spin until back right reads black and back left reads white simultaneously
    {
      Stop();
      delay(500);
      forwards(duty);
      delay(1500);                                                                     //longer delay to allow the robot to enter the corridor
      count++;
    }
    else
    {
      spinClockwise(duty);
    }
  }

  else if(!readGroundSensor(1)&&readGroundSensor(2))                //general line following code
  {
    turnLeft(duty);
  }
  else if(readGroundSensor(1)&&!readGroundSensor(2))
  {
    turnRight(duty);
  }
  else
  {
    forwards(duty);
  }
}  

void challenge4GridRight()
{
                        
  if(count == 0)                    //count is initialised to zero
  {
    forwards(duty);                     //release from the starting square //facing right
    delay(1000);
    count++;                        ////increase turn counter by 1 //repeats after each turn
  }
  else if(count == 1 && readGroundSensor(3) && !readGroundSensor(0))  //when first corner is approached turn left
  {
    Stop();
    delay(500);
    while(readGroundSensor(3))                                       //while back right sensor reads white to ensure full turn is completed
    {
      spinAntiClockwise(duty);
      forwards(duty);
      delay(500);
      count++;                                                           
    }
  }
  else if(count == 2 && !readGroundSensor(1) && !readGroundSensor(2))  //when second junction (count == 2) is approached go straight ahead
  {
    forwards(duty);
    delay(500);
    count++;
  }
  else if(count == 3 && !readGroundSensor(3) && !readGroundSensor(0)) //when third junction (count == 3) is approached turn right
  {
    Stop();
    delay(500);
    if(readGroundSensor(0)&&!readGroundSensor(3))                                       //spin until back right reads black and back left reads white simultaneously
    {
      Stop();
      delay(500);
      forwards(duty);
      delay(500);
      count++;
    }
    else
    {
      spinClockwise(duty);
    }
  }
  else if(count == 4 && !readGroundSensor(3) && !readGroundSensor(0)) //when forth junction (count == 4) is approached turn left
  {
    Stop();
    delay(500);
    if(!readGroundSensor(0)&&readGroundSensor(3))                                       //spin until back left reads black and back right reads white simultaneously
    {
      Stop();
      delay(500);
      forwards(duty);
      delay(1500);                                                                      //longer delay to allow the robot to enter the corridor
      count++;
    }
    else
    {
      spinAntiClockwise(duty);
    }
  }
  
  else if(!readGroundSensor(1)&&readGroundSensor(2))                //general line following code
  {
    turnLeft(duty);
  }
  else if(readGroundSensor(1)&&!readGroundSensor(2))
  {
    turnRight(duty);
  }
  else
  {
    forwards(duty);
  }
}  

void challenge4GridLeftFast()    //if starting on the left
{
                        
  if(count == 0)                    //count is initialised to zero
  {
    forwards(duty);                     //release from the starting square //facing left
    delay(1000);
    count++;                        ////increase turn counter by 1 //repeats after each turn
  }
  else if(count == 1 && !readGroundSensor(3) && readGroundSensor(0))  //when first corner is approached turn right
  {
    while(!readGroundSensor(3))                                       //while back right sensor reads black to ensure full turn is completed
    {
      turnRightFast(duty);
      count++;                                                           
    }
  }
  else if(count == 2 && !readGroundSensor(1) && !readGroundSensor(2))  //when second junction (count == 2) is approached go straight ahead
  {
    forwards(duty);
    delay(500);
    count++;
  }
  else if(count == 3 && !readGroundSensor(3) && !readGroundSensor(0)) //when third junction (count == 3) is approached turn left
  {
    while(!readGroundSensor(0))                                       //while back left sensor reads black to ensure full turn is completed
    {
      turnLeftFast(duty);
      count++;
    }
  }
  else if(count == 4 && !readGroundSensor(3) && !readGroundSensor(0)) //when forth junction (count == 4) is approached turn right
  {
    while(!readGroundSensor(3))                                      ////while back right sensor reads black to ensure full turn is completed
    {
      turnRightFast(duty);
      count++;
    }
  }

  else if(!readGroundSensor(1)&&readGroundSensor(2))                //general line following code
  {
    turnLeft(duty);
  }
  else if(readGroundSensor(1)&&!readGroundSensor(2))
  {
    turnRight(duty);
  }
  else
  {
    forwards(duty);
  }
}

void challenge4GridRightFast() //equivalent to above but reversed
{                    
  if(count == 0)
  {
    forwards(duty);                   
    delay(1000);
    count++;
  }
  else if(count == 1 && readGroundSensor(3) && !readGroundSensor(0)) 
  {
    while(!readGroundSensor(0))
    {
      turnLeftFast(duty);
      count++;
    }
  }
  else if(count == 2 && !readGroundSensor(1) && !readGroundSensor(2))
  {
    delay(500);
    count++;
  }
  else if(count == 3 && !readGroundSensor(3) && !readGroundSensor(0))
  {
    while(!readGroundSensor(3))
    {
      turnRightFast(duty);
      count++;
    }
  }
  else if(count == 4 && !readGroundSensor(3) && !readGroundSensor(0))
  {
    while(!readGroundSensor(0))
    {
      turnLeftFast(duty);
      count++;
    }
  }
  else if(count == 5 && !readGroundSensor(1) && !readGroundSensor(2))
  {
    Stop();
  }  
  else if(!readGroundSensor(1)&&readGroundSensor(2))
  {
    turnLeft(duty);
  }
  else if(readGroundSensor(1)&&!readGroundSensor(2))
  {
    turnRight(duty);
  }
  else
  {
    forwards(duty);
  }
}


void wallFollowingLeft() // if starting on the left of the track
{
  if(count == 5)            // if grid is completed
  {
    if(readProximitySensor(2) > 825)  //hug the right wall
    {
      turnRight(duty);
    }
    else if(readProximitySensor(2) < 750) //give enough room from the wall
    {
      turnLeft(duty);
    }
    else if(!readGroundSensor(0) || !readGroundSensor(3)) //stop when robot has passed through the grid and reaches black finish square
    {
      Stop();
    }
    else
    {
      forwards(duty);                     
    }
  }
}

void wallFollowingRight() // if starting on the right of the track
{
  if(count == 5);             //if grid is completed
  {
    if(readProximitySensor(6) > 825) //hug the left wall 
    {
      turnLeft(duty);
    }
    else if(readProximitySensor(6) < 750) //give enough room from the wall
    {
      turnRight(duty);
    }
    else if(!readGroundSensor(0) || !readGroundSensor(3)) //stop when robot has passed through the grid and reaches black finish square
    {
      Stop();
    }
    else
    {
      forwards(duty);
    }
  } 
}

//SENSOR FUNCTIONS
//Ground sensor functions
int thresh[] = {981, 966, 968, 983}; // black/white thresholds for sensors {0, 1, 2, 3} 
int thresh1[] = {968, 940, 939, 955}; //used to keep track of previous values when tuning


void groundLEDon(unsigned char lineindex)
{
  PORTJ |= (1<<lineindex); //set inputted LED pin to 1 (turn on)
}

void groundLEDoff(unsigned char lineindex)
{
  PORTJ &= (0<<lineindex); //set inputted LED pin to 0 (turn off)
}

int readGroundSensor(unsigned char lineindex) //read value at an inputted ground sensor - return black / white determination
{
    groundLEDon(lineindex);
    delay(5);                          //5 ms delay
    int val = analogRead(lineindex+8); //infrared receivers mapped to ADC channels 8-11 for ground LEDs 0-3 respectively //variable to store read values 
    groundLEDoff(lineindex);
    if(val > thresh[lineindex])         //comparison with tuned threshold values
    {
      return 0; //black
    }
    else
    {
      return 1; //white
    }
}


void readGroundSensors()  //used for tuning thresholds // when rotary selector value > 4
{
  int groundArr[4];         //array to store sensor readings 
  for(int i = 0; i < 4; i ++)       //iteration through each sensor
  {
    groundLEDon(i);
    delay(5);
    groundArr[i] = analogRead(i+8);
    groundLEDoff(i);
    Serial.print("Ground sensor ");       //process same as above except Serial.print to print values to monitor 
    Serial.print(i);
    Serial.print(": ");
    Serial.println(groundArr[i]);
    Serial.print("Ground sensor ");
    Serial.print(i);
    Serial.print(": ");
    if(groundArr[i] > thresh[i])
    {
      Serial.println("Black");
    }
    else
    {
      Serial.println("white");
    }
  }
}

// prox sensor functions
int readProximitySensor(unsigned char proxindex)    // read value at an inputted proximity sensor value
{
    proxLEDon(proxindex);
    delay(5);                                       //5ms delay 
    int val = analogRead(proxindex);                //variable to store read value //sensor pins mapped to digital pins 0-7
    proxLEDoff(proxindex);
    return val;                                     //return read value
}

void proxLEDon(unsigned char proxindex)
{
  PORTA |= (1<<proxindex); //set inputted LED pin to 1 (turn on)
}

void proxLEDoff(unsigned char proxindex)
{
  PORTA &= (0<<proxindex); //set inputted LED pin to 1 (turn off)
}

//MOVEMENT FUNCTIONS
 
void turnRight(float duty)
{
  rightMotorForward(duty*0.5);
  leftMotorForward(duty*1.1);
}

void turnRightFast(float duty)
{
  rightMotorForward(duty*0);
  leftMotorForward(duty);
}

void turnRightSlow(float duty)
{
  rightMotorForward(duty*0.8);
  leftMotorForward(duty*1.1);
}

void turnLeft(float duty)
{
  leftMotorForward(duty*0.7);
  rightMotorForward(duty*1.1);
}

void turnLeftFast(float duty)
{
  leftMotorForward(duty*0);
  rightMotorForward(duty);
}

void turnLeftSlow(float duty)
{
  leftMotorForward(duty);
  rightMotorForward(duty*1.1);
}

void forwards(float duty)
{
  leftMotorForward(duty*1.1);  //wheels turned at different speeds for the same duty cycle - left wheel had to be tuned to acheive a straight line
  rightMotorForward(duty);
}

void backwards (float duty)
{
  leftMotorBackward(duty);
  rightMotorBackward(duty);
}

void Stop()
{
  rightMotorStop();
  leftMotorStop();
}

void spinClockwise(float duty)
{
  leftMotorForward(duty);
  rightMotorBackward(duty);
}

void spinAntiClockwise(float duty)
{
  leftMotorBackward(duty);
  rightMotorForward(duty);
}

void leftMotorForward(float duty)
{
  analogWrite(7, 0); //left motor backwards pin set to duty cycle of 0
  analogWrite(6, duty); //left motor forwards pin set to an inputted duty cycle
  
}

void leftMotorBackward(float duty)
{
  analogWrite(6, 0); //left motor forwards pin set to duty cycle of 0
  analogWrite(7, duty); //left motor backwards pin set to an inputted duty cycle
}

void leftMotorStop()
{
  analogWrite(6, 0); //left motor forwards pin set to duty cycle of 0
  analogWrite(7, 0); //left motor backwards pin set to duty cycle of 0
}
void rightMotorForward(float duty)
{
  analogWrite(2, 0);    //right motor backwards pin set to duty cycle of 0
  analogWrite(5, duty); //right motor forwards pin set to an inputted duty cycle
}

void rightMotorBackward(float duty)
{
  analogWrite(5, 0);    //right motor forwards pin set to duty cycle of 0
  analogWrite(2, duty); //right motor backwards pin set to an inputted duty cycle
  
}

void rightMotorStop()
{
  analogWrite(5, 0);  //right motor forwards pin set to duty cycle of 0
  analogWrite(2, 0);  //right motor backwards pin set to duty cycle of 0
}

#include <Servo.h>
Servo wservo;
Servo xservo;
Servo yservo;
Servo zservo;

// I've just set the default angle to 90 so they all start in the same spot
// change depending on the chassis design
#define DEFAULT_ANGLE  90

// custom servowrite function to clean up code
// dont get confused with Servo.write()
int Servowrite(Servo s,int cur,int prev);

// ch = channel for selecting servos
char ch;
 // cur = current, prev = previous. this is to add smoothing to our movement
int v=0,i=0,xcur=0,xprev=DEFAULT_ANGLE;
int ycur=0,yprev=DEFAULT_ANGLE;
int zcur=0,zprev=DEFAULT_ANGLE;
int wcur=0,wprev=DEFAULT_ANGLE;
// change the delay to change smoothing
int d=10; //delay

void setup() {
  // servo pins can be changed
wservo.attach(6);
xservo.attach(9);
yservo.attach(10);
zservo.attach(11);


Serial.begin(9600);

// I've set them to default to 90
wservo.write(DEFAULT_ANGLE);
xservo.write(DEFAULT_ANGLE);
yservo.write(DEFAULT_ANGLE);
zservo.write(DEFAULT_ANGLE);
Serial.println("Starting");
Serial.println("Enter commands in the format AngleChannel");
Serial.println("e.g. 120x");
}

void loop() {
  if(Serial.available()>0)
   {
    ch=Serial.read();
    switch(ch) 
    {
      case '0': // no break statements so it will fall through to the next one
      case '1':
      case '2':
      case '3':
      case '4':
      case '5':
      case '6':
      case '7':
      case '8':
      case '9':v=v*10+ch-'0'; // converts the number to a int from a char
               break;
      case 'w':wcur=v;
               wprev=Servowrite(wservo,wcur,wprev);
               break;  
      case 'x':xcur=v;
               xprev=Servowrite(xservo,xcur,xprev);
               break;  
      case 'y':ycur=v;
               yprev=Servowrite(yservo,ycur,yprev);
               break;  
      case 'z':zcur=v;
               zprev=Servowrite(zservo,zcur,zprev);               
               break; 
      
      default:break;
    }
   }
}



int Servowrite(Servo s,int cur,int prev)
{
  if(prev<cur)
    { for(i=prev;i<=cur;i++)
        {s.write(i);
         delay(d);
         }
     } 
   else if(cur<prev)
     { for(i=prev;i>=cur;i--)
         {s.write(i);
          delay(d);
         }
     }
     v=0;
     return cur;
}
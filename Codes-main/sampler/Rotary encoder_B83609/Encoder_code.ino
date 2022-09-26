//==========================================================================
//  Author      : Ng Kwang Chen @ MYBOTIC www.mybotic.com.my
//  Project     : RPM Calculator with LCD
//  Date        : 3 Dec 2017       
//==========================================================================
#define ENCODER_PIN 2
#define RESOLUTION 40

unsigned long gulCount = 0;

unsigned long gulStart_Timer = 0;
unsigned short gusChange = 0;
unsigned long gulStart_Read_Timer = 0;
short gsRPM = 0;

void setup() 
{
  // put your setup code here, to run once:

  Serial.print("Angular Velocity");
  Serial.print("    Project");
  delay(2000);
    
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), COUNT, CHANGE);

  Serial.begin(9600);
}

void loop() 
{
  // put your main code here, to run repeatedly:
  if((millis() - gulStart_Read_Timer) >= 500)
  {
    gsRPM = usRead_RPM(); 

    if(gsRPM >= 300)
    {
      gsRPM = gsRPM - 230;
    }
    Serial.print("RPM: ");
    Serial.println(gsRPM);
    gulStart_Read_Timer = millis();
  } 
}

short usRead_RPM(void)
{
  unsigned long ulRPM = 0;
  unsigned long ulTimeDif = 0;

  detachInterrupt(digitalPinToInterrupt(ENCODER_PIN));

  ulTimeDif = millis() - gulStart_Timer;
  ulRPM = 60000*gulCount;
  ulRPM = ulRPM/ulTimeDif;
  ulRPM = ulRPM/RESOLUTION;
  //usRPM = ((60*1000*gulCount)/ulTimeDif)/RESOLUTION;
  
  gulCount = 0;    
  gulStart_Timer = millis();
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), COUNT, CHANGE); 

  return (short)ulRPM;
}

void COUNT(void) 
{ 
  gulCount++;
  gusChange = 1;
}

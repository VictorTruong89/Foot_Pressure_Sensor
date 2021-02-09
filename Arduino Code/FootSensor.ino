// Control pins for Analog Output from sensor's 7 columns
const int col1 = A3;
const int col2 = A6;
const int col3 = A2;
const int col4 = A5;
const int col5 = A1;
const int col6 = A4;
const int col7 = A0;
int output_pin[] = {col1, col2, col3, col4, col5, col6, col7};

// Control pins for Digital Input from sensor's 15 rows
const int row1 = 52;
const int row2 = 53;
const int row3 = 50;
const int row4 = 51;
const int row5 = 48;
const int row6 = 49;
const int row7 = 46;
const int row8 = 47;
const int row9 = 44;
const int row10 = 45;
const int row11 = 42;
const int row12 = 43;
const int row13 = 40;
const int row14 = 41;
const int row15 = 38;
int input_pin[] = {row1, row2, row3, row4, row5,
                    row6, row7, row8, row9, row10,
                    row11, row12, row13, row14, row15};


float temp_reading;           // Temporary buffer to hold & check sensor reading before displaying
unsigned char pressure_mat[15][7] = {0};      // Pressure matrix 

// SERIAL COMMUNICATION parameters
bool startFlag = false;
bool sensorFlag = false;
bool endFlag = false;

//=======================================================================//
void setup()
{  
  // Initialize all Sensor's Rows as voltage input & Set to ZERO
  for(int i = 0; i < 15 ; i++)
  {
    pinMode(input_pin[i], OUTPUT);
    digitalWrite(input_pin[i], LOW);
  }
  // Begin Serial Communication
  Serial.begin(115200);
}

//=======================================================================//
void loop()
{
  StartComm();
  
  if(sensorFlag == true) 
  {
    ReadSensor();
    sensorFlag = false;
    //endFlag = true;
    //delay(100);
  }

  //EndComm();
}

//=======================================================================//
void StartComm()
{
  if (Serial.available() > 0)
  {
    unsigned char serial_command = Serial.read();
    Serial.write(serial_command);
    if(serial_command == 255) {
      sensorFlag = true;
    }      
  }
}

//=======================================================================//
void EndComm()
{
  if(endFlag == true)
  {
    unsigned char end_signal = 255;
    Serial.print(end_signal);
    endFlag == false;
  }
}

//=======================================================================//
void ReadSensor()
{
  // TURN ON INPUT PINS
  for(int i = 0; i < 15; i++)
  {
    digitalWrite(input_pin[i], HIGH);

  // READ OUTPUT PINS
    for(int j = 0; j < 7; j++)
    {
      temp_reading = analogRead(output_pin[j]);
      pressure_mat[i][j] = map(temp_reading, 0, 1023, 0, 254);
      // Some pressure cells are non-existent >> Set to ZEROS
      if(j == 0)
      {
        if(i == 0 || i == 14)             // Cells [1,1] [15,1] are invalid
        {
          pressure_mat[i][j] = 0;
        }
      }
      else if(j == 5 && i == 14)          // Cells [15,6] are invalid
      {
        pressure_mat[i][j] = 0;
      }
      else if(j == 6)
      {
        if(i == 0 || i == 14 || i == 13)  // Cells [1,7] [14,7] [15,7] are invalid
        {
          pressure_mat[i][j] = 0;
        }
      }
  // WRITE THE RESPECTIVE PRESSURE CELLS TO SERIAL PORT
      Serial.write(pressure_mat[i][j]);
    }
      digitalWrite(input_pin[i], LOW);    // Turn off Input Pins
  }
  //Serial.print(pressure_mat);
}

String message; //string that stores the incoming message

void setup()
{

  Serial3.begin(57600); //set baud rate
}

void loop()
{

  Serial3.println("HELLO WORLD");
  
}
    


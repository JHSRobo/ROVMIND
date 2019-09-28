//Simple function to find the power of a component 
//Arduino default implementation uses doubles and leads to round off errors 
int power(int base, int exponent){
  int value = 1;
  for(int i = 0; i < exponent; i++){
    value*=base;
  }
  return value;
}

//test packet: 32 4326 -543 432 654 32 -56 7
//Parse the packet based on spaces and number of values 
inline boolean receivePacket() {
  
  int bitIndex = 0; //Moves through the array in a while loop 
  int startValue = 0; //for substring like implmentation of extracting the integer
  int endValue = 0; //for substring like implmentation of extracting the integer
  
  for (int valueIndex=0; valueIndex < numValues; valueIndex++) { //incrmenets from 0 to numValues - 1 
     startValue = bitIndex; //starting char of each int 
     
     while(bitIndex < packet.length() - 1 && packet.charAt(bitIndex) != ' '){
       bitIndex++; //find the space following each int
     }
     
     if(bitIndex <= packet.length()){ //make sure that the bitIndex does read past the final char in the string
       
       endValue = bitIndex;
       int temp = 0; //hold the integer value as if is converted from char to int
       boolean negative = false; //flag for special negative case

       //custom to int function that uses the start and endvalues(non-inclusive)
       for(int i = startValue; i < endValue; i++){
         if(packet.charAt(i) >= 48 && packet.charAt(i) <= 57){ //make sure that the input is valid
             temp = temp + ((int)packet.charAt(i) - 48)*power(10, endValue - i - 1);
         } else if(packet.charAt(i) == '-' && i == startValue){ //special case for negative characters
           negative = true;
         } else {
           return false; //invalid input
         } 
       }
       if(negative){
           temp*=-1;
         }
         packetInput[valueIndex] = temp; //fill in the input array with integer values
       
     } else {
       return false; //if not all 8 values are read
     }
     bitIndex++; //skip the space
  }
  return true; //successfully parsed the stirng
}

int power(int base, int exponent){
  int value = 1;
  for(int i = 0; i < exponent; i++){
    value*=base;
  }
  return value;
}

inline boolean receivePacket() {
  
  int bitIndex = 0;
  int startValue = 0;
  int endValue = 0;
  
  for (int valueIndex=0; valueIndex < numValues; valueIndex++) {
     startValue = bitIndex;
     while(bitIndex < packet.length() && packet.charAt(bitIndex) != ' '){
       bitIndex++;
     }
     
     if(bitIndex <= packet.length()){
       
       endValue = bitIndex;
       int temp = 0;
       boolean negative = false;
       
       for(int i = startValue; i < endValue; i++){
         if(packet.charAt(i) >= 48 && packet.charAt(i) <= 57){;
             temp = temp + ((int)packet.charAt(i) - 48)*power(10, endValue - i - 1);
         } else if(packet.charAt(i) == '-' && i == startValue){
           negative = true;
         } else {
           return false;
         } 
       }
       if(negative){
           temp*=-1;
         }
         SerialUSB.println(temp);
         packetInput[valueIndex] = 0;
       
     } else {
       return false; //if not all 8 vlaue are read return false
     }
     bitIndex++;
  }
  return true;
}

void send(){
  data_to_be_sent[0]=state;
  data_to_be_sent[1]=acc_estimated[0];
  data_to_be_sent[2]=acc_estimated[1];
  data_to_be_sent[3]=acc_estimated[2];
  data_to_be_sent[4]=angle_estimated[0];
  data_to_be_sent[5]=angle_estimated[1];
  data_to_be_sent[6]=angle_estimated[2];
  for (int i = 0; i < 7; i++) {
    Serial.print(data_to_be_sent[i], 2);
    Serial.print(","); 
  }
  Serial.println();
  }

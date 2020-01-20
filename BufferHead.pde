/*
 bufhead 0 | 1
 bufhead 2 | 3
 */

class BufferHead {
  int id;
  int numBufHeads;
  float[][] bufBuf;

  PVector readLocation;
  PVector readVelocity;
  PVector readAcceleration;
  int readX;
  int readY;

  PVector writeLocation;
  PVector writeVelocity;
  PVector writeAcceleration;
  int writeX;
  int writeY;

  int resolution;  // 1-1024 buffer stepsize
  int bufWidth;

  PVector minLocation;
  PVector maxLocation;

  boolean norm = true;
  int resolutionFrameCounter; // frames since resolution changed

  Gain input;
  Gain[] fbGains;          // read signal of all bufHeads
  Glide[] fbGainGlides;    // read signal of all bufHeads

  Glide remainMixGlide;

  Function bufRead;
  Function bufWrite;

  RMS rms;
  Function getRMS;
  float rmsVal;

  BiquadFilter hpf; 
  Glide hpfGlide;
  BiquadFilter lpf; 
  Glide lpfGlide;


  float tempReadVal;
  float tempWriteVal;

  boolean behind = true;

  float hiThresh;
  float loThresh;

  //float easing;

  BufferHead(float[][] _buf, int _res, int _id) {
    id = _id;
    numBufHeads = 4;
    bufBuf = _buf;
    bufWidth = bufBuf.length;
    resolution = _res;

    hiThresh = 0.3;
    loThresh = 0.15;


    readLocation = new PVector(0, 0);
    readVelocity = new PVector(0, 0);
    readAcceleration = new PVector(0, 0);
    writeLocation = new PVector(0, 0);
    writeVelocity = new PVector(0, 0);
    writeAcceleration = new PVector(0, 0);

    if (id==0) {
      minLocation = new PVector(0, 0);
      maxLocation = new PVector(bufWidth/2, bufWidth/2);
    } else if (id == 1) {
      minLocation = new PVector(bufWidth/2, 0);
      maxLocation = new PVector(bufWidth, bufWidth/2);
    } else if (id == 3) {
      minLocation = new PVector(bufWidth/2, bufWidth/2);
      maxLocation = new PVector(bufWidth, bufWidth);
    } else if (id == 2) {
      minLocation = new PVector(0, bufWidth/2);
      maxLocation = new PVector(bufWidth/2, bufWidth);
    }

    input = new Gain(ac, 1);

    bufRead = new Function(input) {
      public float calculate() {
        // get current buffer value 
        //readX =  constrain(round(readLocation.x/resolution*resolution), 0, bufWidth-1);
        //readY = constrain(round(readLocation.y/resolution*resolution), 0, bufWidth-1);
        readX =  constrain((int)readLocation.x/resolution*resolution, 0, bufWidth-1);
        readY = constrain((int)readLocation.y/resolution*resolution, 0, bufWidth-1);
        tempReadVal = constrain(bufBuf[readX][readY], -2, 2);
        updateLocation(0, readLocation, readVelocity); 

        return tempReadVal;
      }
    };

    remainMixGlide = new Glide(ac, 0, 20);

    bufWrite = new Function(input, remainMixGlide) {

      public float calculate() {
        // get current buffer value and mix with input signal 
        writeX =  constrain((int)writeLocation.x/resolution*resolution, 0, bufWidth-1);
        writeY = constrain((int)writeLocation.y/resolution*resolution, 0, bufWidth-1);
        //writeX =  constrain(round(writeLocation.x/resolution*resolution), 0, bufWidth-1);
        //writeY = constrain(round(writeLocation.y/resolution*resolution), 0, bufWidth-1);
        tempWriteVal = bufBuf[writeX][writeY] * x[1] + x[0];
        //tempWriteVal = bufBuf[writeX][writeY]*remainMix + x[0];
        //tempWriteVal = (bufBuf[writeX][writeY]*remainMix + x[0]) * (1 - random(1)*0.1); // erosion

        //tempWriteVal =  pow(2, -2.0 * abs(tempWriteVal) + 0.1) * tempWriteVal ;  

        //tempWriteVal *= (float)Math.atan(tempWriteVal) * 0.63662 ; 

        tempWriteVal =  pow(2, -1.2 * abs(tempWriteVal) + 0.2) * tempWriteVal ;  

        if (limit) {      // modulo statt limiter 
          //tempWriteVal %= 1;
          //if (tempWriteVal>0.8) {
          //  tempWriteVal  = 0.8 + (tempWriteVal - 0.8) * 0.25;
          //} else if (tempWriteVal<-0.8) {
          //  tempWriteVal  += -0.8 + (tempWriteVal + 0.8) * 0.25;
          //}
          //if (tempWriteVal>1) {
          //  tempWriteVal -= 2;
          //} else if (tempWriteVal<-1) {
          //  tempWriteVal += 2;
          //}
          tempWriteVal = constrain(tempWriteVal, -1.0, 1.0);
        } 

        bufBuf[writeX][writeY] = tempWriteVal; 
        //if (id == 0 || id == 1) bufBuf[writeX][writeY] = tempWriteVal; // test
        updateLocation(1, writeLocation, writeVelocity);

        return tempWriteVal;
      }
    };
    ac.out.addDependent(bufWrite);

    rms = new RMS(ac, 1, 441);
    rms.addInput(bufRead);
    getRMS = new Function(rms) {

      public float calculate() {
        rmsVal = min(1.0, x[0]);
        return x[0];
      }
    };
    ac.out.addDependent(getRMS);

    lpfGlide = new Glide(ac, 10000, 20);
    hpfGlide = new Glide(ac, 20, 20);

    hpf = new BiquadFilter(ac, BiquadFilter.HP, hpfGlide, 1);
    hpf.addInput(bufRead);
    lpf = new BiquadFilter(ac, BiquadFilter.LP, lpfGlide, 1);
    lpf.addInput(hpf);

    fbGainGlides = new Glide[numBufHeads];
    fbGains = new Gain[numBufHeads];
    for (int i=0; i<numBufHeads; i++) {
      fbGainGlides[i] = new Glide(ac, 0, 20);
      fbGains[i] = new Gain(ac, 1, fbGainGlides[i]);
      input.addInput(fbGains[i]);
    }

    //
  }  // end constructor


  void followWriteHead() {
    readVelocity.x += (0.1001 - (easing * 0.1)) * (writeVelocity.x + writeVelocity.x * rw_deviation * 0.01 - readVelocity.x);
  }


  void followFlow(FlowField flow) {  // frame rate
    //What is the vector at that spot in the flow field?
    writeAcceleration.add(flow.lookup(writeLocation).mult(0.1));
    writeVelocity.add(writeAcceleration);
    writeVelocity.mult(0.9999);  // drag
    writeAcceleration.mult(0);
  }


  void updateLocation(int in, PVector location, PVector velocity) {  // audio rate
    if (norm) {
      location.x += velocity.x;
    } else {
      location.x -= velocity.x;
    }
    //location.x += flowField.lookupMag(location) * 0.01;
    if (in == 0) location.x += flowField.lookupMag(location) * 0.01;
    //location.x += velocity.x * resolution;
    //
    while (location.x >= maxLocation.x) {
      location.x -= maxLocation.x - minLocation.x; 
      location.y += resolution;
      while (location.y >= maxLocation.y) {
        //location.y = minLocation.y;
        location.y -= maxLocation.y - minLocation.y;
      }
    } 
    while (location.x < minLocation.x) {
      location.x += maxLocation.x - minLocation.x; 
      location.y -= resolution;
      while (location.y < minLocation.y) {
        //location.y = maxLocation.y - resolution;
        location.y += maxLocation.y - minLocation.y;
      }
    }
    //} 
    //else {  // vertical
    //  //
    //  location.y += velocity.x;
    //  //
    //  while (location.y >= maxLocation.y) {
    //    location.y -= maxLocation.y - minLocation.y; 
    //    location.x += resolution;
    //    while (location.x >= maxLocation.x) {
    //      //location.x = minLocation.x;
    //      location.x -= maxLocation.x - minLocation.x;
    //    }
    //  } 
    //  while (location.y < minLocation.y) {
    //    location.y += maxLocation.y - minLocation.y; 
    //    location.x -= resolution;
    //    while (location.x < minLocation.x) {
    //      //location.x = maxLocation.x - resolution;
    //      location.x += maxLocation.x - minLocation.x;
    //    }
    //  }
    //}
  }


  //
}
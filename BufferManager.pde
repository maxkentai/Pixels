class BufferManager {
  ArrayList <BufferHead> bufHeads;
  int numBufHeads;
  float maxAvgVMag;    // max avarage velo mag of all 4 sectors
  boolean presence;      // if  maxAvgVMag over minAvgPresence
  boolean presenceChanged; // presence state switched
  float maxMaxMag;    // max velo mag of all 4 sectors
  float glitchFrameCounter;  // frames since glitch occured
  int glitchState;   // if glitch or this > 0, then glitch. switch from bufmanager updateparams. 1 micro glitch, 2 glitch
  int glitchType;   //  convolution type
  float resolutionTimeout;    // change resolution after some time
  boolean quantizeGlitch;    // quantize glitch
  float quantizeGlitchFrameCounter;    // quantize glitch

  BufferManager(float[][] _buf, int _res) {
    numBufHeads = 4;
    bufHeads = new ArrayList();
    for (int i=0; i<numBufHeads; i++) {
      bufHeads.add(new BufferHead(_buf, _res, i));
    }

    // connect all fbs
    for (int i=0; i<numBufHeads; i++) {
      for (int j=0; j<numBufHeads; j++) {
        bufHeads.get(i).fbGains[j].addInput(bufHeads.get(j).lpf);
      }
    }
    //
  }


  void setResolution(int which, int _res) {
    BufferHead bH = bufHeads.get(which);
    bH.resolution = _res;
    bH.readLocation.y = (int)bH.readLocation.y / _res * _res;
    bH.writeLocation.y = (int)bH.writeLocation.y / _res * _res;
  }

  void setResolutionAll(int _res, boolean reset) {
    if (reset) {
      if (_res == 1) {
        resolutionTimeout = 500;
      } else if (_res == 2) {
        resolutionTimeout = 5000;
      } else if (_res == 4) {
        resolutionTimeout = 5000;
      } else if (_res == 8) {
        resolutionTimeout = 5000;
      } else if (_res == 16) {
        resolutionTimeout = 5000;
      } else if (_res == 32) {
        resolutionTimeout = 2500;
      } else if (_res == 64) {
        resolutionTimeout = 200;
      } else if (_res == 128) {
        resolutionTimeout = 100;
      } else if (_res == 256) {
        resolutionTimeout = 100;
      } else if (_res == 512) {
        resolutionTimeout = 100;
      } else {
        _res = 4;
        resolutionTimeout = 1000;
      }
    }
    for (BufferHead bH : bufHeads) {
      bH.resolution = _res;
      bH.writeLocation.x = (int)bH.writeLocation.x / _res * _res;
      bH.writeLocation.y = (int)bH.writeLocation.y / _res * _res;
    }
  }


  void setReadVelocity(int which, float x, float y) {
    bufHeads.get(which).readVelocity.x = bufHeads.get(which).readVelocity.x + x * 0.001;
  }

  void setReadVelocityAll(float x, float y) {
    for (BufferHead bH : bufHeads) {
      bH.readVelocity.x = bH.readVelocity.x + x * 0.001;
    }
  }

  void setReadVelocityAllNow(float x, float y) {
    for (BufferHead bH : bufHeads) {
      bH.readVelocity.x =  x;
    }
  }

  void setWriteVelocity(int which, float x, float y) {
    bufHeads.get(which).writeVelocity.x = x;
  }

  void setWriteVelocityAll(float x, float y) {
    for (BufferHead bH : bufHeads) {
      bH.writeVelocity = new PVector(x+(x*random(1)*0.0001), y);
    }
  }


  void setRemainMix(int which, float mix) {
    bufHeads.get(which).remainMixGlide.setValue(mix);
  }

  void setRemainMixAll(float mix) {
    for (BufferHead bH : bufHeads) {
      bH.remainMixGlide.setValue(mix);
    }
  }


  void setSelfFb(int which, float fb) {
    bufHeads.get(which).fbGainGlides[which].setValue(fb);
  }

  void setSelfFbAll(float fb) {
    for (int i=0; i<numBufHeads; i++) {
      bufHeads.get(i).fbGainGlides[i].setValue(fb);
    }
  }


  void setFb(int bufHead, int which, float fb) {
    bufHeads.get(bufHead).fbGainGlides[which].setValue(fb);
  }

  void setCrossFbAll(float fb) {
    for (int i=0; i<numBufHeads; i++) {
      for (int j=0; j<numBufHeads; j++) {
        if (i!=j) bufHeads.get(i).fbGainGlides[j].setValue(fb);
      }
    }
  }


  void setHpf(int which, float freq) {
    bufHeads.get(which).hpfGlide.setValue(freq);
  }

  void setHpfAll(float freq) {
    for (BufferHead bH : bufHeads) {
      bH.hpfGlide.setValue(freq);
    }
  }


  void setLpf(int which, float freq) {
    bufHeads.get(which).lpfGlide.setValue(freq);
  }

  void setLpfAll(float freq) {
    for (BufferHead bH : bufHeads) {
      bH.lpfGlide.setValue(freq);
    }
  }





  // --------------------------------------------------------------
  void updateBufHeadParams() {
    // check highest vmag of all sectors -> presence detection
    maxAvgVMag = 0.0;
    maxMaxMag = 0.0;
    for (int i=0; i< flowField.avgVMags.length; i++) {
      maxAvgVMag = max(flowField.avgVMags[i], maxAvgVMag);
      maxMaxMag = max(flowField.maxMags[i], maxMaxMag);
    }
    //
    if (maxAvgVMag < minAvgPresence) {  // no presence in all sectors
      if (presence) {
        presence = false;
        presenceChanged = true;
      } else {
        presenceChanged = false;
      }
      //for (BufferHead bH : bufHeads) {
      //  //bH.writeVelocity.x *= 0.999;
      //  //if (bH.writeVelocity.x < 1)  bH.writeVelocity.x = 1;
      //}
    } else { // presence detected
      if (!presence) {
        presence = true;
        presenceChanged = true;
        boolean pass = true;

        for (int i = 0; i < numBufHeads; i++) {
          if (bufHeads.get(i).rmsVal > 0.01) {
            pass = false;
            break;
          }
        }

        if (pass || resolutionTimeout <= 0) {
          resolution = newResolution(resolution, 1, 5);
          setResolutionAll(resolution, true);
          bufHeads.get((int)random(4)).writeVelocity.x *= random(0.9, 1.1);
        }
        quantizeGlitch = false;
        quantizeGlitchFrameCounter = 0;
      } else {
        presenceChanged = false;
      }
    }
    //  Glitch
    if (glitchFrameCounter <= 0) { // glitch out
      if (glitchState == 1) {
        setResolutionAll(resolution, false);  // reset resolution no timer reset      
        glitchState = 0;
      } else if (glitchState == 2) {
        resolution = newResolution(resolution, 1, 5);
        setResolutionAll(resolution, true);      
        glitchState = 0;
        if (quantizeGlitchFrameCounter <= 0) {
          quantizeGlitch = true;
          quantizeGlitchFrameCounter = 2500;
        } else {
          quantizeGlitch = !quantizeGlitch;
        }
      }
    } else {  // glitching
      if (glitchState == 2 && glitchFrameCounter% 20 == 0) {
        resolution = newResolution(resolution, 0, 8);
        setResolutionAll(resolution, false);
      }
    }
    //
    if (maxMaxMag > maxActivity ) {  // 
      if (glitchFrameCounter <= 0) {  // check for glitch condition, avoid retriggering
        //glitchFrameCounter = (int)((maxMaxMag - maxActivity) * 30);  // reset glitch frame counter
        if (maxMaxMag - maxActivity > 5) {
          glitchFrameCounter = 200;  // set glitch frame counter
          glitchState = 2;
          glitchType = 0;
          //glitchType = (int)random(2);
        } else {
          glitchFrameCounter = 10;  // set glitch frame counter
          glitchState = 1;
        }
      }
    } 

    //
    for (BufferHead bH : bufHeads) {

      float cGain = 0;

      // remain mix
      cGain = bH.remainMixGlide.getValue();  
      if (bH.rmsVal < bH.loThresh && flowField.avgVMags[bH.id] > minAvgPresence) {  // below low threshold and presence
        cGain += 0.005;
      } else {
        cGain *= 0.997;
      }
      cGain = constrain(cGain, 0, 1);
      bH.remainMixGlide.setValue(cGain);

      // y velo vector
      if (flowField.avgPV[bH.id].y > 0) {
        bH.readVelocity.x += flowField.avgVMags[bH.id]*0.0005*bH.resolution;
        //bH.readVelocity.x += flowField.avgVMags[bH.id]*0.01;
      } else {
        bH.readVelocity.x -= flowField.avgVMags[bH.id]*0.0005*resolution;
        //bH.readVelocity.x -= flowField.avgVMags[bH.id]*0.01;
      }

      // x velo vector
      if (flowField.avgPV[bH.id].x > 0) {
        //bH.writeVelocity.x += flowField.avgVMags[bH.id]*0.01;
        if (flowField.avgPV[bH.id].x > 1) bH.norm = true;
      } else {
        //bH.writeVelocity.x -= flowField.avgVMags[bH.id]*0.01;
        if (flowField.avgPV[bH.id].x < -1) bH.norm = false;
      }

      if (flowField.avgVMags[bH.id] > minAvgPresence) {
        //bH.writeVelocity.x *= 1.1;
        //if (bH.writeVelocity.x > 9.46)  bH.writeVelocity.x = 9.46;
      }



      // rms dependent adaptation
      //
      //if (bH.rmsVal > 0.4) {  // max level
      //  bH.hiThresh -= (bH.rmsVal - 0.4)*0.1;
      //  bH.hiThresh = max(bH.loThresh + 0.1, bH.hiThresh);
      //} else {
      //  bH.hiThresh += 0.0005;
      //  bH.hiThresh = min(bH.hiThresh, 0.3);
      //}

      if (bH.rmsVal > 0.40 ) {  
        if (bH.resolutionFrameCounter>2 && resolutionTimeout < 1000) {  // 2 frames since last local res change
          //setResolution(bH.id, (int)pow(2, (int)random(9)));
          setResolution(bH.id, newResolution(bH.resolution, 0, 8));
          bH.resolutionFrameCounter = 0;
        }
      }

      // check if new resolution to be set: upper limit
      if (bH.rmsVal > 0.35 ) {  
        if (resolutionTimeout <= 0) {
          boolean pass = true;
          for (int i = 0; i < numBufHeads; i++) {
            if (i != bH.id && bufHeads.get(i).rmsVal < 0.35) {
              pass = false;
              break;
            }
          }
          if (pass) {
            quantizeGlitch = false;
            resolution = newResolution(resolution, 1, 5);
            setResolutionAll(resolution, true);
            bufHeads.get((int)random(4)).writeVelocity.x *= random(0.9, 1.1);
          }
        }
      }


      if (bH.rmsVal > bH.hiThresh || flowField.avgVMags[bH.id] < minAvgPresence) {  // above high threshold or no presence
        //if (bH.rmsVal > bH.hiThresh ) {  // tttt above high threshold or no presence
        //bH.readVelocity.x *= 0.99999; // testtest
        // if read - write > x -> easing decrease

        for (int i = 0; i < numBufHeads; i++) {
          if (bufHeads.get(i).rmsVal > bH.hiThresh) {
            cGain = bH.fbGainGlides[i].getValue();
            cGain *= 0.995;
            bH.fbGainGlides[i].setValue(cGain);
          }
        }
      } 

      if (flowField.avgVMags[bH.id] < minAvgPresence) {  // above high threshold or no presence

        for (int i = 0; i < numBufHeads; i++) {
          if (flowField.avgVMags[i] < minAvgPresence) {
            cGain = bH.fbGainGlides[i].getValue();
            cGain *= 0.999;
            //cGain = max(0, cGain - 0.002);
            bH.fbGainGlides[i].setValue(cGain);
          }
        }
      } 


      //if (bH.rmsVal < bH.hiThresh && flowField.avgVMags[bH.id] > minAvgPresence) {  // below high threshold and presence
      if (bH.rmsVal < bH.loThresh ) {  // tttt below high threshold and presence
        if (bH.resolution != resolution) {
          if (resolutionTimeout <= 0) {
            if (bH.resolution == 1 || bH.resolution > 32) resolution = newResolution(resolution, 1, 5);
            else resolution = bH.resolution;
            setResolutionAll(bH.resolution, true);
            quantizeGlitch = false;
          } // new resolution
          else  setResolution(bH.id, resolution);  // reset resolution
          bH.norm = true;
        }

        if (flowField.avgVMags[bH.id] > minAvgPresence) {
          for (int i = 0; i < numBufHeads; i++) {
            //if (bufHeads.get(i).rmsVal < bufHeads.get(i).hiThresh) {  // tttt
            if (bufHeads.get(i).rmsVal < bufHeads.get(i).hiThresh && flowField.avgVMags[i] > minAvgPresence) {
              cGain = bH.fbGainGlides[i].getValue();
              cGain += 0.001;
              cGain = constrain(cGain, 0, 1);
              bH.fbGainGlides[i].setValue(cGain);
            }
          }
        }

        //bH.loThresh += 0.0001;
        //bH.loThresh = min(bH.hiThresh - 0.1, bH.loThresh);
        //bH.easing -= 0.0001;
        //bH.easing = max(0, bH.easing);
      } else {
        //bH.loThresh -= 0.0001;
        //bH.loThresh = max(bH.loThresh, 0.01);
        //bH.easing += 0.0001;
        //bH.easing = min(1, easing);
      }


      // check if new resolution to be set: lower limit
      //if (bH.rmsVal < 0.1 ) {
      //  if (resolutionTimeout <= 0 && maxAvgVMag > minAvgPresence) {  // time out and presence detected
      //    boolean pass = true;

      //    for (int i = 0; i < numBufHeads; i++) {
      //      if (i != bH.id && bufHeads.get(i).rmsVal > 0.1) {
      //        pass = false;
      //        break;
      //      }
      //    }
      //    if (pass) {
      //      resolution = newResolution(resolution, 1, 5);
      //      setResolutionAll(resolution);
      //      bufHeads.get((int)random(4)).writeVelocity.x *= random(0.9, 1.1);
      //    }
      //  }
      //}


      bH.resolutionFrameCounter++;
    } // end loop bufHeads
    //
    glitchFrameCounter -= (60.0/frameRate);
    quantizeGlitchFrameCounter -= (60.0/frameRate);
    resolutionTimeout -= (60.0/frameRate);
  }  // end update params


  int newResolution(int _res, int lo, int hi) {
    int res = (int)pow(2, (int)random(lo, hi+1)); 
    while (res == _res) {
      res = (int)pow(2, (int)random(lo, hi+1));
    }
    return res;
  }

  void followWriteHead() {
    for (BufferHead bH : bufHeads) {
      bH.followWriteHead();
    }
  }

  void alignWriteHeadVelocities() {
    for (int i=0; i<bufHeads.size(); i++) {
      float otherVelosX = 0;
      for (int j=0; j<bufHeads.size(); j++) {
        if (i!=j) {
          otherVelosX += bufHeads.get(j).writeVelocity.x;
        }
      }
      otherVelosX = otherVelosX / (bufHeads.size()-1);
      bufHeads.get(i).writeVelocity.x += (0.1001 - (easing * 0.1)) * (otherVelosX + otherVelosX*w_deviation*pow(-1, i) - bufHeads.get(i).writeVelocity.x);
    }
  }

  void followFlow(FlowField flow) {
    for (BufferHead bH : bufHeads) {
      bH.followFlow(flow);
    }
  }

  float getRmsAtLocation(float x, float y) {
    return bufHeads.get(int(x / bufWidth * 2) + int(y / bufWidth * 2) * 2).rmsVal;
  }

  int getResolutionAtLocation(float x, float y) {
    return bufHeads.get((int)x / (bufWidth/2)  + (int)y / (bufWidth/2) * 2).resolution;
  }
}  // end buffermanager
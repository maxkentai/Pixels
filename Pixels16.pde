/* //<>//
1.03  bufBuf[][], class bufferHead, read follows write
 1.05 dead end
 1.06 copy of 1.03
 1.07 readVelo relativ zu writeVelo
 1.08 rms~fläche test, vertical write test, faun2d
 1.09 4-channel test, video to buffer mit ff velocity
 1.10 4-channel version, interaction test
 1.11 fbGain[], interaction
 1.12 256x256 flowField test
 1.13 interaction parameter tuning with presence detection (avgvmag > x)
 1.14 master präsi
 1.15 anorg sgmk
 1.16 lsd75
 
 // Beads Buffer.buf default buffer size is 4096
 */


import org.openkinect.freenect.*;
import org.openkinect.processing.*;
import beads.*;
import org.jaudiolibs.beads.AudioServerIO;
import java.util.Arrays; 
import com.thomasdiewald.pixelflow.java.DwPixelFlow;
import com.thomasdiewald.pixelflow.java.imageprocessing.DwOpticalFlow;
import com.thomasdiewald.pixelflow.java.imageprocessing.filter.DwFilter;
import controlP5.*;


AudioContext ac;
IOAudioFormat audioFormat;
float sampleRate = 44100;
int buffer = 512;
int bitDepth = 16;
int inputs = 2;
int outputs = 6;
boolean stereo = false;  // stereo or quad

ControlP5 cp5;

Kinect kinect;
boolean kinectConnected;
boolean irMode = true;
int selBufHead = 4; // gui parameter affect selected bufhead settings, 4 = all

float[][] bufBuf;
boolean[][] bufBufDesc;  // test for faun2d
int bufWidth = 1024;
int bufSize = bufWidth*bufWidth;
int resolutionExp = 3;
int resolution;


PImage depthImg;  // thresholded kinect depth image
PImage videoImg;  // kinect video image
PImage constrImg;  // 256x256 test, 128x128 mapped kinect image


DwPixelFlow context;
DwOpticalFlow opticalflow;
PGraphics2D pg_cam;  // this graphic will be analyzed by pixelflow
int   pgImgW;    // image size for pg_cam image for flowtracking. equal to flowField dimensions
FlowField flowField;
int flowFieldResolution = 4;  // 8, in pixels
int frameSkip = 2;    // get velocity vectors from pixelflow and update flowField every n frame
int camSkip = 1;    // 2, use every nth pixel of kinect depth image 640x480
// float buffer for pixel transfer from OpenGL to the host application
float[] flow_velocity;
PImage imgBlur; // blurred image of img3
PGraphics imgMouse; // used for mouse input drawing


PGraphics tp1; 
PGraphics tp2; 
int tpNbr;
int tpRes = 16;


int minDepth =  550;

//int maxDepthL = 1008;
//int maxDepthU = 1078;
int maxDepthL = 971;
int maxDepthU = 1020;

boolean[][] inFramePixels;
Preference myPrefs = new Preference();
PVector[] projectionPoints = new PVector[4];  // 0= upper left, 1= upper right, 2= lower right, 3= lower left ----> clockwise
int pMinX, pMinY, pMaxX, pMaxY;  // min/max values calc from projectionPoints

float remainMix = 0;
float crossFb = 0;
float selfFb = 0;

PImage img;    // the image to display and buffer for everything
PImage imgDisp;


boolean convolve = false;
boolean convolveX = false;
boolean convolveY=false;
boolean dither;
boolean faun2d;

int bright = 255;
boolean glitch=false;
boolean convWrap;  // convolve wrap around
int view_w = 1024;
int view_h = 1024;
boolean START_FULLSCREEN = true;
boolean limit = true;
boolean quantize = true;

BufferManager bufManager;

Slider volumeSlider;
float volume = 0.5;  // slider value
float maxVolume = 0.1;
Glide volGlide;


Slider remainMixSlider;
Slider crossFbSlider;
Slider selfFbSlider;

Slider resoSlider;

Slider lpfSlider;
Slider hpfSlider;
float lpfFreq = 10000;
float hpfFreq = 20;

Slider deviationSlider;
float w_deviation = 0.0;

Slider rw_deviationSlider;
float rw_deviation = 0.0;

Slider easingSlider;
float easing = 0.89; // 0-1

Slider readXIncSlider;
Slider writeXIncSlider;

float readXInc = 0.0;
float writeXInc = 13.3113;
//float writeXInc = 9.46;


// interaction params to adjust
float minActivity = 1;  // local vmag for interaction
float maxActivity = 15.0;  // max mag for triggering glitchstate 
float minAvgPresence = 0.0001;  // average vmag for presence detection
float minAvgActivity = 1.00;  // average vmag for activity detection
//


int imgXOffset;
int imgYOffset;
int guiXOff;
int guiYOff;

boolean printscreen = false;
boolean debug = false;
boolean setupMode = false;
boolean freeze;
boolean record;
boolean showFlowField;
boolean showKinectImage;
boolean align = true;

WavePlayer sine;
Glide sineFreqGlide;
int bufIndX;
int bufIndY;

float perlZOff;



public void settings() {
  fullScreen(P2D, 1);
}


void setup() {
  kinect = new Kinect(this);
  kinect.initDepth();
  kinect.initVideo();
  if (Kinect.countDevices() > 0) {
    kinectConnected = true;
    kinect.enableIR(irMode);
  }
  imgXOffset = (width-bufWidth)/2; // for the psoitioning of the image on the canvas
  imgYOffset = (height-bufWidth)/2;
  guiXOff =  2;
  //guiXOff = (width + bufWidth)/2 + 20;
  guiYOff = height/2;

  inFramePixels = new boolean[kinect.width][kinect.height];
  // load Preferences
  if (myPrefs.loadPref() == 0) {  // loaded ok
    for (int i = 0; i < projectionPoints.length; i++) {
      projectionPoints[i] = new PVector(myPrefs.getFloat("p"+i+"x"), myPrefs.getFloat("p"+i+"y"));
    }
    //threshold = myPrefs.getFloat("threshold");
  } else {
    projectionPoints[0] = new PVector(10, 10);
    projectionPoints[1] = new PVector(kinect.width - 10, 10);
    projectionPoints[2] = new PVector(kinect.width - 10, kinect.height - 10);
    projectionPoints[3] = new PVector(10, kinect.height - 10);
  }
  calcProjPixels();


  tp1 = createTestPattern(0);
  tp2 = createTestPattern(1);

  if (stereo) {
    ac = new AudioContext(128);
  } else {
    audioFormat = new IOAudioFormat(sampleRate, bitDepth, inputs, outputs);
    ac = new AudioContext(new AudioServerIO.Jack(), buffer, audioFormat);
  }


  img = new PImage(bufWidth, bufWidth, RGB);
  imgDisp = new PImage(bufWidth, bufWidth, RGB);
  bufBuf = new float[bufWidth][bufWidth];
  bufBufDesc = new boolean[bufWidth][bufWidth];
  resolution = (int)pow(2, resolutionExp);

  flowField = new FlowField(flowFieldResolution, bufWidth, bufWidth);
  println(flowField.cols + " "  + flowField.rows);
  pgImgW = flowField.cols;  // 256 test, 128
  flow_velocity = new float[pgImgW * pgImgW * 2]; // *2 because x and y velocity

  bufManager = new BufferManager(bufBuf, resolution);
  bufManager.setWriteVelocityAll(writeXInc, writeXInc);
  bufManager.setReadVelocityAllNow(writeXInc - 0.001, 0);

  bufManager.setCrossFbAll(crossFb);
  bufManager.setSelfFbAll(selfFb);
  bufManager.setRemainMixAll(remainMix);

  volGlide = new Glide(ac, 0, 50);
  volGlide.setValue(volume * maxVolume);

  Gain[] outs = new Gain[4];
  for (int i = 0; i<4; i++) {
    //outs[i] = new Gain(ac, 1, 0.05);
    outs[i] = new Gain(ac, 1, volGlide);
    outs[i].addInput(bufManager.bufHeads.get(i).bufRead);
  }

  if (stereo) {
    ac.out.addInput(1, outs[0], 0);  // connect bufferreader to output
    ac.out.addInput(1, outs[1], 0);  // connect bufferreader to output
    ac.out.addInput(0, outs[2], 0);  // connect bufferreader to output
    ac.out.addInput(0, outs[3], 0);  // connect bufferreader to output
  } else {
    ac.out.addInput(0, outs[0], 0);  // connect bufferreader to output
    ac.out.addInput(1, outs[1], 0);  // connect bufferreader to output
    ac.out.addInput(3, outs[3], 0);  // connect bufferreader to output
    ac.out.addInput(2, outs[2], 0);  // connect bufferreader to output
  }

  //sineFreqGlide = new Glide(ac, 100, 1);
  //sine = new WavePlayer(ac, sineFreqGlide, Buffer.SINE);

  //Function writeSine = new Function(sine) {
  //  public float calculate() {
  //    float mag = flowField.getShapedVelocityMagAtLocation(new PVector(bufIndX, bufIndY));
  //    if (mag > 0.1) {
  //      bufBuf[bufIndX][bufIndY] = x[0];
  //      sineFreqGlide.setValue(mag*2000);
  //    }
  //    bufIndX+=resolution;
  //    if (bufIndX >= bufWidth) {
  //      bufIndX = 0;
  //      bufIndY+=resolution;
  //      if (bufIndY >= bufWidth) {
  //        bufIndY = 0;
  //      }
  //    }
  //    return x[0];
  //  }
  //};
  //ac.out.addDependent(writeSine);
  ac.start();


  // --------------------------------------------

  cp5 = new ControlP5(this);
  if (!debug) cp5.setVisible(false);

  volumeSlider = cp5.addSlider("volume")
    .setPosition(guiXOff, guiYOff - 10)
    .setRange(0, 1.0)
    ;
  volumeSlider.addCallback(new CallbackListener() {
    public void controlEvent(CallbackEvent theEvent) {
      if (theEvent.getAction()==ControlP5.ACTION_BROADCAST) {
        volGlide.setValue(volumeSlider.getValue() * maxVolume);
      }
    }
  }
  );


  remainMixSlider = cp5.addSlider("remainMix")
    .setPosition(guiXOff, guiYOff)
    .setRange(0, 1.0)
    ;
  remainMixSlider.addCallback(new CallbackListener() {
    public void controlEvent(CallbackEvent theEvent) {
      if (theEvent.getAction()==ControlP5.ACTION_BROADCAST) {
        if (selBufHead < 4) bufManager.setRemainMix(selBufHead, remainMixSlider.getValue());
        bufManager.setRemainMixAll(remainMixSlider.getValue());
      }
    }
  }
  );

  crossFbSlider = cp5.addSlider("crossFb")
    .setPosition(guiXOff, guiYOff + 10)
    .setRange(0, 1.0)
    ;
  crossFbSlider.addCallback(new CallbackListener() {
    public void controlEvent(CallbackEvent theEvent) {
      if (theEvent.getAction()==ControlP5.ACTION_BROADCAST) {
        //if (selBufHead < 4) bufManager.setCrossFb(selBufHead, crossFbSlider.getValue());
        bufManager.setCrossFbAll(crossFbSlider.getValue());
      }
    }
  }
  );

  selfFbSlider = cp5.addSlider("selfFb")
    .setPosition(guiXOff, guiYOff + 20)
    .setRange(0, 1.0)
    ;
  selfFbSlider.addCallback(new CallbackListener() {
    public void controlEvent(CallbackEvent theEvent) {
      if (theEvent.getAction()==ControlP5.ACTION_BROADCAST) {
        if (selBufHead < 4) bufManager.setSelfFb(selBufHead, selfFbSlider.getValue());
        else bufManager.setSelfFbAll(selfFbSlider.getValue());
      }
    }
  }
  );

  resoSlider = cp5.addSlider("resolutionExp")
    .setPosition(guiXOff, guiYOff + 40)
    .setRange(0, 10)
    ;
  resoSlider.addCallback(new CallbackListener() {
    public void controlEvent(CallbackEvent theEvent) {
      if (theEvent.getAction()==ControlP5.ACTION_BROADCAST) {
        resolution = (int)pow(2, (int)resoSlider.getValue());
        if (selBufHead < 4) bufManager.setResolution(selBufHead, resolution);
        else  bufManager.setResolutionAll(resolution, true);
      }
    }
  }
  );


  // -------------------------
  readXIncSlider = cp5.addSlider("readXInc")
    .setPosition(guiXOff, guiYOff + 70)
    .setRange(-1, 1)
    ;
  readXIncSlider.addCallback(new CallbackListener() {
    public void controlEvent(CallbackEvent theEvent) {
      if (theEvent.getAction()==ControlP5.ACTION_BROADCAST) {
        if (selBufHead < 4) bufManager.setReadVelocity(selBufHead, readXIncSlider.getValue(), readXIncSlider.getValue());
        else bufManager.setReadVelocityAll(readXIncSlider.getValue(), readXIncSlider.getValue());
      }
    }
  }
  );

  writeXIncSlider = cp5.addSlider("writeXInc")
    .setPosition(guiXOff, guiYOff + 80)
    .setRange(-100, 100)
    ;
  writeXIncSlider.addCallback(new CallbackListener() {
    public void controlEvent(CallbackEvent theEvent) {
      if (theEvent.getAction()==ControlP5.ACTION_BROADCAST) {
        if (selBufHead < 4) bufManager.setWriteVelocity(selBufHead, writeXIncSlider.getValue(), writeXIncSlider.getValue());
        else bufManager.setWriteVelocityAll(writeXIncSlider.getValue(), writeXIncSlider.getValue());
      }
    }
  }
  );


  deviationSlider = cp5.addSlider("w_deviation")
    .setPosition(guiXOff, guiYOff + 90)
    .setRange(0, 1)
    ;
  deviationSlider.addCallback(new CallbackListener() {
    public void controlEvent(CallbackEvent theEvent) {
      if (theEvent.getAction()==ControlP5.ACTION_BROADCAST) {
        w_deviation =  deviationSlider.getValue() * 0.001;
      }
    }
  }
  );


  rw_deviationSlider = cp5.addSlider("rw_deviation")
    .setPosition(guiXOff, guiYOff + 100)
    .setRange(-0.5, 0.5)
    ;
  rw_deviationSlider.addCallback(new CallbackListener() {
    public void controlEvent(CallbackEvent theEvent) {
      if (theEvent.getAction()==ControlP5.ACTION_BROADCAST) {
        rw_deviation = floor(rw_deviationSlider.getValue()*100)/100.0;
      }
    }
  }
  );

  easingSlider = cp5.addSlider("easing")
    .setPosition(guiXOff, guiYOff + 110)
    .setRange(0, 1.0)
    ;

  hpfSlider = cp5.addSlider("hpfFreq")
    .setPosition(guiXOff, guiYOff + 140)
    .setRange(20, 5000)
    ;
  hpfSlider.addCallback(new CallbackListener() {
    public void controlEvent(CallbackEvent theEvent) {
      if (theEvent.getAction()==ControlP5.ACTION_BROADCAST) {
        if (selBufHead < 4) bufManager.setHpf(selBufHead, hpfSlider.getValue());
        else bufManager.setHpfAll(hpfSlider.getValue());
      }
    }
  }
  );

  lpfSlider = cp5.addSlider("lpfFreq")
    .setPosition(guiXOff, guiYOff + 150)
    .setRange(20, 10000)
    ;
  lpfSlider.addCallback(new CallbackListener() {
    public void controlEvent(CallbackEvent theEvent) {
      if (theEvent.getAction()==ControlP5.ACTION_BROADCAST) {
        if (selBufHead < 4) bufManager.setLpf(selBufHead, lpfSlider.getValue());
        else bufManager.setLpfAll(lpfSlider.getValue());
      }
    }
  }
  );



  // ----------------------------------  

  println("kinectConnected: " + kinectConnected);

  // Blank image
  depthImg = new PImage(kinect.width, kinect.height);
  videoImg = new PImage(kinect.width, kinect.height);
  constrImg = new PImage(pgImgW, pgImgW);
  // main library context
  context = new DwPixelFlow(this);
  context.print();
  context.printGL();

  // optical flow
  opticalflow = new DwOpticalFlow(context, pgImgW, pgImgW);
  pg_cam = (PGraphics2D) createGraphics(pgImgW, pgImgW, P2D);  // 256x256
  imgMouse = createGraphics(pgImgW, pgImgW, P2D);  // for simulating cam input with the mouse

  //
}





void draw() {
  background(0);

  flowTracking();
  bufManager.followWriteHead();
  bufManager.updateBufHeadParams();
  if (align) bufManager.alignWriteHeadVelocities();

  if (setupMode) {  
    noFill();
    stroke(255);
    rect(0, 0, width-1, height-1);
    image(constrImg, (width-bufWidth)/2, (height-bufWidth)/2, bufWidth, bufWidth);
    if (tpNbr == 1) image(tp1, (width-bufWidth)/2, (height-bufWidth)/2);
    else if (tpNbr == 2) image(tp2, (width-bufWidth)/2, (height-bufWidth)/2);
    if (showFlowField) {
      if (tpNbr == 1 || tpNbr == 0) stroke(255);
      else stroke(0);
      pushMatrix();
      translate((width-bufWidth)/2, (height-bufWidth)/2);
      flowField.display();
      popMatrix();
    }
    rect((width-bufWidth)/2, (height-bufWidth)/2, bufWidth, bufWidth);
    if (showKinectImage) {
      pushMatrix();
      stroke(255);
      translate((width-(2*kinect.width))/2, (height-kinect.height)/2);
      image(videoImg, 0, 0);
      rect(0, 0, kinect.width, kinect.height);
      translate(kinect.width, 0);
      image(depthImg, 0, 0);
      rect(0, 0, kinect.width, kinect.height);
      quad(
        projectionPoints[0].x, 
        projectionPoints[0].y, 
        projectionPoints[1].x, 
        projectionPoints[1].y, 
        projectionPoints[2].x, 
        projectionPoints[2].y, 
        projectionPoints[3].x, 
        projectionPoints[3].y 
        );
      popMatrix();
    }
  } else {  // not setupMode
    if (quantize) quantizePixels();
    if (convolve || bufManager.glitchState > 0) convolve();
    // display the image
    image(img, imgXOffset, imgYOffset);
  }

  if (debug) {
    textSize(24);
    if (bufManager.presence) fill(0, 255, 0);
    else fill(255, 0, 0);
    text("maxAvgVMag: " + String.format("%.5f", bufManager.maxAvgVMag), guiXOff, guiYOff - 230);
    fill(255);
    text( "maxMaxMag: "+ String.format("%.5f", bufManager.maxMaxMag), guiXOff, guiYOff - 200);
    text("glitchState: " + bufManager.glitchState + " quantGlitch: " + bufManager.quantizeGlitch, guiXOff, guiYOff - 170);
    text("glFrame: " + (int)bufManager.glitchFrameCounter + " resT-out: "+ (int)bufManager.resolutionTimeout, guiXOff, guiYOff - 140);
    text("framerate: " + floor(frameRate), guiXOff, guiYOff - 110);
    text("Resolution: " + resolution, guiXOff, guiYOff - 80);
    textSize(12);
    text("align: "+ align + " bufHeadNbr: "+selBufHead, guiXOff, guiYOff + 200);
    text("convolve: "+convolve+" X: "+convolveX+" Y: "+convolveY + " dither: "+ dither + " faun: "+ faun2d, guiXOff, guiYOff + 210);
    text("quantize: "+quantize+" limit: "+limit+ " glitch: "+glitch, guiXOff, guiYOff + 220);

    text("min: "+ minDepth+ " maxDepthL: "+ maxDepthL+ " maxDepthU: "+maxDepthU, guiXOff, guiYOff + 230);

    //
    text("read__0: "+ (bufManager.bufHeads.get(0).readVelocity.x), guiXOff, guiYOff + 250);
    text("write_0: "+ (bufManager.bufHeads.get(0).writeVelocity.x), guiXOff, guiYOff + 260);
    text("thresh_0: "+ String.format("%.2f", bufManager.bufHeads.get(0).loThresh)+" / " + String.format("%.2f", bufManager.bufHeads.get(0).hiThresh), guiXOff, guiYOff + 270);
    fill(0, 255, 0);
    text("rms_0: "+ String.format("%.3f", bufManager.bufHeads.get(0).rmsVal), guiXOff, guiYOff + 280);
    fill(255);
    text("fb_0/0: "+ String.format("%.2f", bufManager.bufHeads.get(0).fbGainGlides[0].getValue()), guiXOff, guiYOff + 290);
    text("0/1: "+ String.format("%.2f", bufManager.bufHeads.get(0).fbGainGlides[1].getValue()), guiXOff + 80, guiYOff + 290);
    text("fb_0/2: "+ String.format("%.2f", bufManager.bufHeads.get(0).fbGainGlides[2].getValue()), guiXOff, guiYOff + 300);
    text("0/3: "+ String.format("%.2f", bufManager.bufHeads.get(0).fbGainGlides[3].getValue()), guiXOff + 80, guiYOff + 300);

    text("remn_0: "+ String.format("%.2f", bufManager.bufHeads.get(0).remainMixGlide.getValue()), guiXOff, guiYOff + 310);
    text("hpf_0: "+ bufManager.bufHeads.get(0).hpfGlide.getValue(), guiXOff, guiYOff + 330);
    text("lpf_0: "+ bufManager.bufHeads.get(0).lpfGlide.getValue(), guiXOff, guiYOff + 340);
    text("vAvg_0: "+ String.format("%.5f", flowField.avgVMags[0]), guiXOff, guiYOff + 350);
    text("vMax_0: "+ String.format("%.5f", flowField.maxMags[0]), guiXOff, guiYOff + 360);
    text("vPV_0: "+ String.format("%.2f", flowField.avgPV[0].x) +" / " + String.format("%.2f", flowField.avgPV[0].y), guiXOff, guiYOff + 370);
    //     
    text("read__1: "+ (bufManager.bufHeads.get(1).readVelocity.x), guiXOff+160, guiYOff + 250);
    text("write_1: "+ (bufManager.bufHeads.get(1).writeVelocity.x), guiXOff+160, guiYOff + 260);
    text("thresh_1: "+ String.format("%.2f", bufManager.bufHeads.get(1).loThresh)+" / " + String.format("%.2f", bufManager.bufHeads.get(1).hiThresh), guiXOff+160, guiYOff + 270);
    fill(0, 255, 0);
    text("rms_1: "+ String.format("%.3f", bufManager.bufHeads.get(1).rmsVal), guiXOff+160, guiYOff + 280);
    fill(255);
    text("fb_1/0: "+ String.format("%.2f", bufManager.bufHeads.get(1).fbGainGlides[0].getValue()), guiXOff+160, guiYOff + 290);
    text("1/1: "+ String.format("%.2f", bufManager.bufHeads.get(1).fbGainGlides[1].getValue()), guiXOff+160 + 80, guiYOff + 290);
    text("fb_1/2: "+ String.format("%.2f", bufManager.bufHeads.get(1).fbGainGlides[2].getValue()), guiXOff+160, guiYOff + 300);
    text("1/3: "+ String.format("%.2f", bufManager.bufHeads.get(1).fbGainGlides[3].getValue()), guiXOff+160 + 80, guiYOff + 300);

    text("remn_1: "+ String.format("%.2f", bufManager.bufHeads.get(1).remainMixGlide.getValue()), guiXOff+160, guiYOff + 310);
    text("hpf_1: "+ bufManager.bufHeads.get(1).hpfGlide.getValue(), guiXOff+160, guiYOff + 330);
    text("lpf_1: "+ bufManager.bufHeads.get(1).lpfGlide.getValue(), guiXOff+160, guiYOff + 340);
    text("vAvg_1: "+ String.format("%.5f", flowField.avgVMags[1]), guiXOff+160, guiYOff + 350);
    text("vMax_1: "+ String.format("%.5f", flowField.maxMags[1]), guiXOff+160, guiYOff + 360);
    text("vPV_1: "+ String.format("%.2f", flowField.avgPV[1].x) +" / " + String.format("%.2f", flowField.avgPV[1].y), guiXOff+160, guiYOff + 370);
    //    
    text("read__2: "+ (bufManager.bufHeads.get(2).readVelocity.x), guiXOff, guiYOff + 390);
    text("write_2: "+ (bufManager.bufHeads.get(2).writeVelocity.x), guiXOff, guiYOff + 400);
    text("thresh_2: "+ String.format("%.2f", bufManager.bufHeads.get(2).loThresh)+" / " + String.format("%.2f", bufManager.bufHeads.get(2).hiThresh), guiXOff, guiYOff + 410);
    fill(0, 255, 0);
    text("rms_2: "+ String.format("%.3f", bufManager.bufHeads.get(2).rmsVal), guiXOff, guiYOff + 420);
    fill(255);
    text("fb_2/0: "+ String.format("%.2f", bufManager.bufHeads.get(2).fbGainGlides[0].getValue()), guiXOff, guiYOff + 430);
    text("2/1: "+ String.format("%.2f", bufManager.bufHeads.get(2).fbGainGlides[1].getValue()), guiXOff + 80, guiYOff + 430);
    text("fb_2/2: "+ String.format("%.2f", bufManager.bufHeads.get(2).fbGainGlides[2].getValue()), guiXOff, guiYOff + 440);
    text("2/3: "+ String.format("%.2f", bufManager.bufHeads.get(2).fbGainGlides[3].getValue()), guiXOff + 80, guiYOff + 440);

    text("remn_2: "+ String.format("%.2f", bufManager.bufHeads.get(2).remainMixGlide.getValue()), guiXOff, guiYOff + 450);
    text("hpf_2: "+ bufManager.bufHeads.get(2).hpfGlide.getValue(), guiXOff, guiYOff + 470);
    text("lpf_2: "+ bufManager.bufHeads.get(2).lpfGlide.getValue(), guiXOff, guiYOff + 480);
    text("vAvg_2: "+ String.format("%.5f", flowField.avgVMags[2]), guiXOff, guiYOff + 490);
    text("vMax_2: "+ String.format("%.5f", flowField.maxMags[2]), guiXOff, guiYOff + 500);
    text("vPV_2: "+ String.format("%.2f", flowField.avgPV[2].x) +" / " + String.format("%.2f", flowField.avgPV[2].y), guiXOff, guiYOff + 510);
    //     
    text("read__3: "+ (bufManager.bufHeads.get(3).readVelocity.x), guiXOff+160, guiYOff + 390);
    text("write_3: "+ (bufManager.bufHeads.get(3).writeVelocity.x), guiXOff+160, guiYOff + 400);
    text("thresh_3: "+ String.format("%.2f", bufManager.bufHeads.get(3).loThresh)+" / " + String.format("%.2f", bufManager.bufHeads.get(3).hiThresh), guiXOff+160, guiYOff + 410);
    fill(0, 255, 0);
    text("rms_3: "+ String.format("%.3f", bufManager.bufHeads.get(3).rmsVal), guiXOff+160, guiYOff + 420);
    fill(255);
    text("fb_3/0: "+ String.format("%.2f", bufManager.bufHeads.get(3).fbGainGlides[0].getValue()), guiXOff+160, guiYOff + 430);
    text("3/1: "+ String.format("%.2f", bufManager.bufHeads.get(3).fbGainGlides[1].getValue()), guiXOff+160 + 80, guiYOff + 430);
    text("fb_3/2: "+ String.format("%.2f", bufManager.bufHeads.get(3).fbGainGlides[2].getValue()), guiXOff+160, guiYOff + 440);
    text("3/3: "+ String.format("%.2f", bufManager.bufHeads.get(3).fbGainGlides[3].getValue()), guiXOff+160 + 80, guiYOff + 440);

    text("remn_3: "+ String.format("%.2f", bufManager.bufHeads.get(3).remainMixGlide.getValue()), guiXOff+160, guiYOff + 450);
    text("hpf_3: "+ bufManager.bufHeads.get(3).hpfGlide.getValue(), guiXOff+160, guiYOff + 470);
    text("lpf_3: "+ bufManager.bufHeads.get(3).lpfGlide.getValue(), guiXOff+160, guiYOff + 480);
    text("vAvg_3: "+ String.format("%.5f", flowField.avgVMags[3]), guiXOff+160, guiYOff + 490);
    text("vMax_3: "+ String.format("%.5f", flowField.maxMags[3]), guiXOff+160, guiYOff + 500);
    text("vPV_3: "+ String.format("%.2f", flowField.avgPV[3].x) +" / " + String.format("%.2f", flowField.avgPV[3].y), guiXOff+160, guiYOff + 510);
    //    
    //
  }
} // end draw



void calcProjPixels() {
  pMinX = (int)min(projectionPoints[0].x, projectionPoints[3].x);
  pMinY = (int)min(projectionPoints[0].y, projectionPoints[1].y);
  pMaxX = (int)(max(projectionPoints[1].x, projectionPoints[2].x));
  pMaxY = (int)(max(projectionPoints[2].y, projectionPoints[3].y));
  // calc in frame pixels
  PVector p;
  for (int x = 0; x < kinect.width; x++ ) {
    for (int y = 0; y < kinect.height; y++ ) {
      p = translateCamPoint(x, y, kinect.width, kinect.height);
      if ((p.x >= 0) && (p.x < kinect.width) && (p.y >= 0) && (p.y < kinect.height)) {
        inFramePixels[x][y] = true;
      } else {
        inFramePixels[x][y] = false;
      }
    }
  }
}


PVector translateCamPoint (float x, float y, int w, int h) {
  PVector[] crossPoints = new PVector[4];
  // calculate x of left crosspoint 
  crossPoints[0] = new PVector (projectionPoints[0].x + (projectionPoints[3].x - projectionPoints[0].x) / (projectionPoints[3].y - projectionPoints[0].y) * (y - projectionPoints[0].y), y);
  // calculate x of right crosspoint 
  crossPoints[1] = new PVector (projectionPoints[1].x + (projectionPoints[2].x - projectionPoints[1].x) / (projectionPoints[2].y - projectionPoints[1].y) * (y - projectionPoints[1].y), y);
  // calculate y of upper crosspoint *--------|-------*
  crossPoints[2] = new PVector (x, projectionPoints[0].y + (projectionPoints[1].y - projectionPoints[0].y) / (projectionPoints[1].x - projectionPoints[0].x) * (x - projectionPoints[0].x));
  // calculate y of lower crosspoint *--------|-------*
  crossPoints[3] = new PVector (x, projectionPoints[3].y + (projectionPoints[2].y - projectionPoints[3].y) / (projectionPoints[2].x - projectionPoints[3].x) * (x - projectionPoints[3].x));

  float tX = (x - crossPoints[0].x) / (crossPoints[1].x - crossPoints[0].x) * w;
  float tY = (y - crossPoints[2].y) / (crossPoints[3].y - crossPoints[2].y) * h;

  return new PVector(tX, tY);
}


void savePrefs() {
  for (int i = 0; i < projectionPoints.length; i++) {
    myPrefs.setNumber("p"+i+"x", projectionPoints[i].x, false);
    myPrefs.setNumber("p"+i+"y", projectionPoints[i].y, false);
  }
  //myPrefs.setNumber("threshold", threshold, false);
  myPrefs.savePref();
}


PGraphics createTestPattern(int type) {
  PGraphics tp = createGraphics(bufWidth+1, bufWidth+1);
  tp.beginDraw();
  if (type == 0)  tp.background(0);
  else tp.background(255);
  tp.noFill();
  tp.strokeWeight(1);
  if (type == 0)  tp.stroke(255);
  else tp.stroke(0);
  for (int i=0; i<tp.width; i+=tpRes) {
    tp.line(i, 0, i, bufWidth);
    tp.line(0, i, bufWidth, i);
  }
  tp.endDraw();
  return tp;
}



// ---------------------------------------------------------------
void flowTracking() {

  constrImg.loadPixels();
  color col = color(0);
  for (int i = 0; i < constrImg.pixels.length; i++) {
    constrImg.pixels[i] = col;
  }

  if (kinectConnected) {
    // Threshold the depth image
    int[] rawDepth = kinect.getRawDepth();
    if (showKinectImage) depthImg.loadPixels();

    for (int x=pMinX; x < pMaxX; x+=camSkip) {
      for (int y=pMinY; y < pMaxY; y+=camSkip) {
        float amt = (float)y/ kinect.height;
        int pos = y * kinect.width + x;
        if (rawDepth[pos] >= minDepth && rawDepth[pos] <= lerp(maxDepthU, maxDepthL, amt)) {
          if (inFramePixels[x][y]) {
            PVector pv = translateCamPoint(x, y, pgImgW, pgImgW);
            constrImg.pixels[(int)pv.y * pgImgW + (int)pv.x] = color(255);
            //constrImg.pixels[(int)pv.y * pgImgW + (int)pv.x] = color(brightness(videoImg.pixels[pos])*10);
          } 
          if (showKinectImage) depthImg.pixels[pos] = color(255);
        } else {
          if (showKinectImage) depthImg.pixels[pos] = color(0);
        }
      }
    }
  }
  constrImg.updatePixels();

  if (mousePressed) {
    noStroke();
    fill(255);
    imgMouse.beginDraw();
    imgMouse.clear();
    imgMouse.rectMode(CENTER);
    imgMouse.rect((mouseX  - imgXOffset) * pgImgW / bufWidth, (mouseY - imgYOffset) * pgImgW / bufWidth, 32, 32);
    imgMouse.endDraw();
    constrImg.blend(imgMouse, 0, 0, pgImgW, pgImgW, 0, 0, pgImgW, pgImgW, ADD);
  }
  if (showKinectImage) depthImg.updatePixels();

  // render to offscreenbuffer
  pg_cam.beginDraw();
  pg_cam.clear();
  pg_cam.image(constrImg, 0, 0);
  pg_cam.endDraw();

  // update Optical Flow
  opticalflow.update(pg_cam); 

  // flow visualizations
  opticalflow.param.display_mode = 0;
  opticalflow.param.blur_input = 25;
  opticalflow.param.temporal_smoothing = 0.5;  // .93

  // Transfer velocity data from the GPU to the host-application
  // This is in general a bad idea because such operations are very slow. So 
  // either do everything in shaders, and avoid memory transfer when possible, 
  // or do it very rarely. however, this is just an example for convenience.
  if (frameCount % frameSkip == 0) {
    flow_velocity = opticalflow.getVelocity(flow_velocity);
    flowField.update(flow_velocity);    // update flowField with current flow velos
  } // end update flowField


  // write to buffer depending ff velocity
  for (int x=0; x < bufWidth; x+= resolution) {
    for (int y=0; y < bufWidth; y+= resolution) {
      float vMag = flowField.getShapedVelocityMagAtLocation(new PVector(x, y));
      float rms = bufManager.getRmsAtLocation(x, y);
      if (vMag > minActivity && rms < 0.05) {
        //if (vMag > minVMag && abs(bufBuf[x][y]) < 0.05) {
        //if (vMag > minVMag ) {
        //bufBuf[x][y] = constrain((noise(x * 0.05, y * 0.05, perlZOff) * 2.0 - 1.0) * vMag/20 + bufBuf[x][y], -1, 1); //
        bufBuf[x][y] = bufBuf[x][y] + vMag/20 * random(-1, 1) * (0.05 - rms) * 20; //
        //bufBuf[x][y] += colorToSignal(constrImg.pixels[i])*vMag/10.0; //
        bufBuf[x][y] =  pow(2, -1.2 * abs(bufBuf[x][y]) + 0.2) * bufBuf[x][y] ;
      }
    }
  } // end loop
  //perlZOff += 0.05;
  //
  // } // end kinect connected
}   // end flow tracking


void convolve() {
  int xstart = 0; 
  int ystart = resolution;
  int xend = img.width;
  int yend = img.height-(2*resolution);
  int matrixsize = 3;

  img.loadPixels();

  if (convolveX || bufManager.glitchType == 0) { 
    // Begin our loop for every pixel
    for (int x = xstart; x < xend; x+=resolution ) {
      for (int y = 0; y < img.height; y+=resolution ) {
        //for (int y = ystart; y < yend; y+=bufManager1.resolution ) {
        // Each pixel location (x,y) gets passed into a function called convolution()
        // The convolution() function returns a new color to be displayed.
        color c = convolutionX(x, y, matrixsize, img); 
        int loc = x + y*img.width;
        img.pixels[loc] = c;
        bufBuf[x][y] = colorToSignal(c);
      }
    }
  }

  if (convolveY || bufManager.glitchType == 1) {
    for (int x = xstart; x < xend; x+=resolution ) {
      for (int y = ystart; y < yend; y+=resolution ) {
        // Each pixel location (x,y) gets passed into a function called convolution()
        // The convolution() function returns a new color to be displayed.
        color c = convolutionY(x, y, matrixsize, img); 
        int loc = x + y*img.width;
        img.pixels[loc] = c;
        bufBuf[x][y] = colorToSignal(c);
      }
    }
  }

  if (dither || bufManager.glitchType == 2) {
    for (int y = 0; y < img.width; y+=resolution ) {
      for (int x = 0; x < img.width; x+=resolution ) {

        int index = index(x, y );
        color pix = img.pixels[index];
        float oldR = red(pix);
        float oldG = green(pix);
        float oldB = blue(pix);
        int factor = 7;
        int newR = round(factor * oldR / 255) * (255/factor);
        int newG = round(factor * oldG / 255) * (255/factor);
        int newB = round(factor * oldB / 255) * (255/factor);
        img.pixels[index] = color(newR, newG, newB);
        bufBuf[x][y] = colorToSignal(img.pixels[index]);

        float errR = oldR - newR;
        float errG = oldG - newG;
        float errB = oldB - newB;


        index = index(x+resolution, y  );
        color c = img.pixels[index];
        float r = red(c);
        float g = green(c);
        float b = blue(c);
        r = r + errR * 7/16.0;
        g = g + errG * 7/16.0;
        b = b + errB * 7/16.0;
        img.pixels[index] = color(r, g, b);
        bufBuf[(x+resolution)%img.width][y] = colorToSignal(img.pixels[index]);

        index = index(x-resolution, y+resolution  );
        c = img.pixels[index];
        r = red(c);
        g = green(c);
        b = blue(c);
        r = r + errR * 3/16.0;
        g = g + errG * 3/16.0;
        b = b + errB * 3/16.0;
        img.pixels[index] = color(r, g, b);
        int n =0;
        if (x-resolution < 0) n = img.width;
        bufBuf[x-resolution + n][(y+resolution)%img.width] = colorToSignal(img.pixels[index]);

        index = index(x, y+resolution);
        c = img.pixels[index];
        r = red(c);
        g = green(c);
        b = blue(c);
        r = r + errR * 5/16.0;
        g = g + errG * 5/16.0;
        b = b + errB * 5/16.0;
        img.pixels[index] = color(r, g, b);
        bufBuf[x][(y+resolution)%img.width] = colorToSignal(img.pixels[index]);


        index = index(x+resolution, y+resolution);
        c = img.pixels[index];
        r = red(c);
        g = green(c);
        b = blue(c);
        r = r + errR * 1/16.0;
        g = g + errG * 1/16.0;
        b = b + errB * 1/16.0;
        img.pixels[index] = color(r, g, b);
        bufBuf[(x+resolution)%img.width][(y+resolution)%img.width] = colorToSignal(img.pixels[index]);
      }
    }
  }
  //
  if (faun2d || bufManager.glitchType == 3) {
    for (int y = 0; y < img.width; y+=resolution ) {
      for (int x = 0; x < img.width; x+=resolution ) {
        float val = 0;
        for (int i = 0; i < 2*resolution; i+=resolution ) {
          int xloc = x + i - resolution;
          if (xloc < 0) {
            xloc += img.width;
          } else if (xloc >= img.width) {
            xloc -= img.width;
          }
          xloc = constrain(xloc, 0, img.width-1);
          //
          for (int j = 0; j < 2*resolution; j+=resolution ) {
            int yloc = y + j - resolution;

            if (yloc < 0) yloc += img.height;
            else if (yloc >= img.height) yloc -= img.height;
            yloc = constrain(yloc, 0, img.height-1);

            //
            if (i==resolution && j==resolution) {
              //val += bufBuf[xloc][yloc];
            } else val += bufBuf[xloc][yloc];
            //if (i==resolution) val += bufBuf[x][yloc];
            //else val += bufBuf[x][yloc]*0.0001;
          }
          //val = pow(2, -1.2 * abs(val) + 0.2);
          val /= 8;
          if (abs(val) > 0.95) {
            bufBuf[x][y] *= 0.9;
            bufBufDesc[x][y] = true;  // descending
          } else  if (abs(val) < 0.05) {
            bufBufDesc[x][y] = false;
            bufBuf[x][y] *= 1.1;
          } else {
            if (bufBufDesc[x][y]) bufBuf[x][y] *= 0.9;
            else bufBuf[x][y] *=  1.1;
          }
          bufBuf[x][y] = constrain(bufBuf[x][y], -1, 1);
          img.pixels[x + y*img.width] = signalToColor(val);
        }
      }
    }
  }

  //
  img.updatePixels();
}

int index(int x, int y) {
  x %= img.width;
  y %= img.width;
  if (x<0) x+= img.width;
  if (y<0) y+= img.width;
  return x + y * bufWidth;
}

color convolutionX(int x, int y, int matrixsize, PImage img) {
  float rtotal = 0.0;
  float gtotal = 0.0;
  float btotal = 0.0;
  int offset = matrixsize / 2;
  float val1 = 0;
  float val2 = 0;
  float val = 0;

  for (int i = 0; i < 3*resolution; i+=resolution ) {
    int xloc = x + i*resolution-resolution;
    if (xloc < 0) xloc += img.width;
    else if (xloc > img.width-resolution) xloc = xloc - img.width;  
    int loc = constrain(xloc + img.width*y, 0, img.pixels.length-1);

    rtotal += (red(img.pixels[loc]) * 1.0);
    gtotal += (green(img.pixels[loc]) * 1.0);
    btotal += (blue(img.pixels[loc]) * 1.0);
    if (i == 0) {
      val1 = red(img.pixels[loc]);
    } else if (i==1) {
      val = red(img.pixels[loc]);
    } else {
      val2 = red(img.pixels[loc]);
    }
  }
  if (val < max(val1, val2)) {
    rtotal = (max(val1, val2) - val) * 0.05 + val;
  } else if (val >= max(val1, val2)) {
  }

  // Make sure RGB is within range
  rtotal = constrain(rtotal, 0, 255);
  gtotal = constrain(rtotal, 0, 255);
  btotal = constrain(rtotal, 0, 255);

  // Return the resulting color
  return color(rtotal, gtotal, btotal);
}


color convolutionY(int x, int y, int matrixsize, PImage img) {
  float rtotal = 0.0;
  float gtotal = 0.0;
  float btotal = 0.0;
  int offset = matrixsize / 2;

  for (int j = 0; j < matrixsize; j++ ) {
    int yloc = y + j*resolution-offset*resolution;
    if (yloc < 0) yloc += img.height;
    else if (yloc > img.height-resolution) yloc = yloc - img.height+resolution;  

    int loc = x + img.width*yloc;
    rtotal += (red(img.pixels[loc]) * 1.0);
    gtotal += (green(img.pixels[loc]) * 1.0);
    btotal += (blue(img.pixels[loc]) * 1.0);
  }

  // Make sure RGB is within range
  rtotal = constrain(rtotal / 3, 0, 255);
  gtotal = constrain(gtotal / 3, 0, 255);
  btotal = constrain(btotal / 3, 0, 255);

  // Return the resulting color
  return color(rtotal, gtotal, btotal);
}




//void updateImage() {  // transfer values from buffer to the imgage's pixels array
//  img.loadPixels();
//  for (int x = 0; x < bufWidth; x++) {
//    for (int y = 0; y < bufWidth; y++) {
//      img.pixels[y*bufWidth+x] = signalToColor(bufBuf[x][y]);
//    }
//  }
//  img.updatePixels();
//}


void quantizePixels() {  // quantise buffer and transfer values to the imgage's pixels array
  img.loadPixels();
  float val;
  color col;
  int res = 0;
  for (int k = 0; k<4; k++) {
    res = bufManager.bufHeads.get(k).resolution;
    int minX = k % 2 * (bufWidth/2);
    int maxX = bufWidth/2 + minX;
    int minY = k / 2 * (bufWidth/2);
    int maxY = bufWidth/2 + minY;

    for (int x = minX; x < maxX; x+=res) {
      for (int y = minY; y < maxY; y+=res) {
        res = bufManager.getResolutionAtLocation(x, y);

        if (glitch || bufManager.quantizeGlitch) res = veloRes(x, y); 
        val  = bufBuf[x][y];
        col = signalToColor(val);
        for (int i = 0; i < res; i++) {
          for (int j = 0; j < res; j++) {
            if ( (x+i) < bufWidth && (y+j) < bufWidth) {
              bufBuf[x+i][y+j] = val;
              img.pixels[(y+j)*bufWidth+x+i] = col;
            }
          }
        }
      }
      //
    }
  }

  img.updatePixels();
}



//void quantizePixels() {  // quantise buffer and transfer values to the imgage's pixels array
//  img.loadPixels();
//  float val;
//  color col;
//  //bufHeads.get(int(x / bufWidth * 2) + int(y / bufWidth * 2) * 2).resolution;

//  int res = resolution;
//  //res = bufManager.getResolutionAtLocation(0, 0);
//  for (int x = 0; x < bufWidth; x+=res) {
//    for (int y = 0; y < bufWidth; y+=res) {
//      res = bufManager.getResolutionAtLocation(x, y);

//      if (glitch) res = randomRes(x, y); 
//      val  = bufBuf[x][y];
//      col = signalToColor(val);
//      for (int i = 0; i < res; i++) {
//        //res = randomRes();
//        for (int j = 0; j < res; j++) {
//          if ( (x+i) < bufWidth && (y+j) < bufWidth) {
//            bufBuf[x+i][y+j] = val;
//            //if (i>0 || j>0) bufBuf[x+i][y+j] = val;
//            img.pixels[(y+j)*bufWidth+x+i] = col;
//          }
//        }
//      }
//    }
//    //
//  }
//  img.updatePixels();
//}



int veloRes(int x, int y) {
  //if (random(100) > 99) {
  //  return (int)pow(2, round(random(resolutionExp)));
  //} else return resolution;
  int exp = 0;
  float vMag = flowField.getShapedVelocityMagAtLocation(new PVector(x, y));
  resolutionExp = (int)(log(resolution)/log(2));
  if (vMag > minActivity) {
    exp = round(resolutionExp - constrain(vMag - minActivity, 1, resolutionExp));
  } else { 
    exp = resolutionExp;
  }
  //if (vMag > minActivity) exp = round(resolutionExp - constrain(vMag - minActivity, 1, resolutionExp));
  //else exp = resolutionExp;
  return (int)pow(2, exp);
}


color signalToColor(float val) {
  //if (val < 0) {
  //  return color((-sqrt(abs(val)) + 1) * 127.5);
  //} else {
  //  return color((sqrt(val) + 1) * 127.5);
  //}
  return color((constrain(val, -0.5, 0.5) + 0.5) * 255);
  //return color((val+1.0) * 127.5);
}

float colorToSignal(color col) {
  return red(col)/127.5-1.0;
}


void mousePressed() { 
  if (showKinectImage && setupMode) {
    float recDist = 10000;
    int recIndex = 0; 
    PVector mouse = new PVector(mouseX-(width-2*kinect.width)/2 - kinect.width, mouseY-(height-kinect.height)/2);
    for (int i = 0; i < projectionPoints.length; i++) {
      float dist = PVector.dist(projectionPoints[i], mouse);
      if (dist < recDist) { 
        recDist = dist;
        recIndex = i;
      }
    }
    projectionPoints[recIndex] = mouse;
    calcProjPixels();
  }
}


void keyPressed() {
  if (key == CODED) {
    if (keyCode == UP) {
    } else if (keyCode == DOWN) {
    }
  } else if (key == 'i') {
    irMode = !irMode;
    kinect.enableIR(irMode);
  } else if (key == 't') {
    tpNbr = (tpNbr+1) % 3;
  } else if (key == 'k') {
    showKinectImage = !showKinectImage;
    if (!showKinectImage) savePrefs();
  } else if (key == 'r') {
    maxDepthU = constrain(maxDepthU+1, minDepth, 2047);
  } else if (key == 'e') {
    maxDepthU = constrain(maxDepthU-1, minDepth, 2047);
  } else if (key == 'w') {
    maxDepthL = constrain(maxDepthL+1, minDepth, 2047);
  } else if (key =='q') {
    maxDepthL = constrain(maxDepthL-1, minDepth, 2047);
  } else  if (key == 'x') {
    convolveX = !convolveX;
  } else  if (key == 'y') {
    convolveY = !convolveY;
  } else  if (key == 'c') {
    convolve = !convolve;
  } else  if (key == 'g') {
    glitch = !glitch;
  } else  if (key == 'l') {
    limit = !limit;
  } else  if (key == 'n') {
    if (selBufHead < 4) bufManager.bufHeads.get(selBufHead).norm = !bufManager.bufHeads.get(selBufHead).norm;
  } else  if (key == 'd') {
    debug = !debug;
    if (debug) {
      cp5.setVisible(true);
      cursor();
    } else {
      cp5.setVisible(false);
      noCursor();
    }
  } else  if (key == 's') {
    setupMode = !setupMode;
    if (!setupMode) {
      showKinectImage = false;
      savePrefs();
    }
  } else  if (key == 'f') {
    showFlowField = !showFlowField;
    faun2d = !faun2d;
  } else  if (key == 'm') {
    quantize = !quantize;
  } else  if (key == '-') {
    align = !align;
  } else  if (key == 'v') {
    dither = !dither;
  } else  if (key == '0') {
    selBufHead = 0;
  } else  if (key == '1') {
    selBufHead = 1;
  } else  if (key == '2') {
    selBufHead = 2;
  } else  if (key == '3') {
    selBufHead = 3;
  } else  if (key == '4') {
    selBufHead = 4;
  }
}

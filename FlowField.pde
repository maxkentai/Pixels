// The Nature of Code
// Daniel Shiffman
// http://natureofcode.com

// Flow Field Following

class FlowField {

  // A flow field is a two dimensional array of PVectors
  PVector[][] field;  // added velo vectors
  float[][] vMags;      //  actual velocities from pixelflow library passed in with flow_velo[]
  float[][] vMagsShaped;  // shaped
  float vDecay = 0.95;// 0.93
  //PVector[][] kernel;  
  color[][] blur; // not used
  int cols, rows; // Columns and Rows
  int resolution; // How large is each "cell" of the flow field
  //int kernelSize;
  float vMag;
  float createVelo = 3.0;  // create particle if velo greater this value and nbr of particles lower max
  float minVelo = 0.1; // 1.0

  float[] avgVMags;       // avarage shaped velo mag for 4 sectors
  float[] avgVMagsTemp;   // avarage shaped velo mag for 4 sectors for calc
  float[] maxMags;        // max velo mag for 4 sectors
  float[] maxMagsTemp;    // max velo mag for 4 sectors for calc
  PVector[] avgPV;        // avarage vector for 4 sectors
  PVector[] avgPVTemp;        // avarage vector for 4 sectors
  float maxMag = 40;

  FlowField(int res_, int width_, int height_) {
    resolution = res_;
    //kernelSize = 3;
    // Determine the number of columns and rows based on sketch's width and height
    cols = (width_ - 1) /resolution + 1;
    rows = (height_ - 1) /resolution + 1;
    field = new PVector[cols][rows];
    vMags = new float[cols][rows];
    vMagsShaped = new float[cols][rows];
    //kernel = new PVector[kernelSize][kernelSize];
    blur = new color[cols][rows];
    avgVMags = new float[4];
    maxMags = new float[4];
    avgVMagsTemp = new float[4];
    maxMagsTemp = new float[4];
    avgPV = new PVector[4];
    avgPVTemp = new PVector[4];
    init();

    println("flowField: ", cols, rows);
  }

  void init() {
    for (int i = 0; i < cols; i++) {
      for (int j = 0; j < rows; j++) {
        field[i][j] = new PVector(0, 0);
        //field[i][j] = new PVector(width/2-i*resolution, height/2-j*resolution);
        field[i][j].normalize();
      }
    }
    for (int i = 0; i < 4; i++) {
      avgPV[i] = new PVector(0, 0);
      avgPVTemp[i] = new PVector(0, 0);
    }

    //for (int i = 0; i < kernelSize; i++) {
    //  for (int j = 0; j <  kernelSize; j++) {
    //    //kernel[i][j] = new PVector(1 - i, 1 - j);
    //    kernel[i][j] = new PVector(1 - i, 1 - j).mult(-1);
    //    kernel[i][j].normalize();
    //  }
    //}
  }



  void update(float[] flow_velo) {  // flow_velo has 2x size of flowField for separate x and y values
    // update flowField with pixelflow vectors
    for (int i=0; i< avgVMagsTemp.length; i++) {
      avgVMagsTemp[i] = 0.0;
      maxMagsTemp[i] = 0.0;
      avgPVTemp[i].mult(0);
    }

    int col2 = cols / 2;
    for ( int x = 0; x < flowField.cols; x++) {
      for ( int y = 0; y < flowField.rows; y++) {

        int PIDX    = min((flow_velo.length - 1) / 2, (pgImgW - 1 - y) * pgImgW + x);
        PVector v = new PVector(flow_velo[PIDX * 2], -flow_velo[PIDX * 2 + 1]);  // flow velocity vector
        vMag = v.mag();  // magnitude of current velo works better
        vMags[x][y] = vMag;

        // shape current velocity
        if (vMag >= vMagsShaped[x][y] && vMag >= 0.001) {
          vMagsShaped[x][y] = min(maxMag, vMag);  // limit and store current velo
        } else {
          vMagsShaped[x][y] = max(0, vMagsShaped[x][y] * vDecay);  // exp decay it
          //vMagsShaped[x][y] = max(0, vMagsShaped[x][y] - vDecay);; // linear decay it
        }
        avgVMagsTemp[y/col2*2+x/col2] += vMagsShaped[x][y];
        maxMagsTemp[y/col2*2+x/col2] = max(maxMagsTemp[y/col2*2+x/col2], vMagsShaped[x][y]);

        // update flowfield  
        if (vMag >= minVelo) {
          flowField.field[x][y].add(v.mult(0.2));  // 0.2
          flowField.field[x][y].limit(maxMag);    // limit to max velo
        }
        flowField.field[x][y].mult(0.995);  // velo is decaying
        avgPVTemp[y/col2*2+x/col2].add(flowField.field[x][y]);
      }
    }

    for (int i=0; i< avgVMags.length; i++) {
      avgVMags[i] = avgVMagsTemp[i] / (cols * cols) * 4;
      maxMags[i] = maxMagsTemp[i];
      avgPV[i] = avgPVTemp[i].copy().div(cols*cols/4);
    }

    // end update flowField





    /*
    // spread current flow vectors to adjectant vectors according to the angle
     for (int x = 1; x < flowField.cols -  1; x++) {
     for (int y = 1; y < flowField.rows - 1; y++) {
     //PVector sum = new PVector(0, 0); // Kernel sum for this pixel
     
     for (int kx = -1; kx <= 1; kx++) {
     for (int ky = -1; ky <= 1; ky++) {
     if (kx != 0 && ky != 0) {
     float angle = PVector.angleBetween(flowField.kernel[kx+1][ky+1], flowField.field[x][y]);
     //float angle = PVector.angleBetween(flowField.field[x][y], flowField.field[x+kx][y+ky]);
     //float angle = PVector.angleBetween(flowField.field[x+kx][y+ky], flowField.field[x][y]);
     angle = abs(angle - PI) / PI;
     if (angle > 0.5 ) {
     PVector pv = PVector.mult(flowField.field[x][y], angle * angle * 0.01);
     //PVector pv = PVector.mult(flowField.field[x][y], angle * 0.2);
     //PVector pv = PVector.mult(flowField.field[x][y], pow(angle, 2) * 0.1);
     flowField.field[x+kx][y+ky].add(pv);
     flowField.field[x+kx][y+ky].limit(3);
     //flowField.field[x+kx][y+ky].add(PVector.mult(flowField.field[x][y], angle));
     }
     }
     }
     }
     }
     }
     */
  } // end update

  // Draw every vector
  void display() {
    for (int i = 0; i < cols; i++) {
      for (int j = 0; j < rows; j++) {
        //drawVector(currentVelos[i][j], i*resolution, j*resolution, resolution*0.2);
        drawVector(field[i][j], i*resolution, j*resolution, resolution*0.2);
      }
    }
  }

  //void spreadVelo(int x, int y, PVector pv) {
  //  if (x>0 && y>0 && x<flowField.cols-1 && y<flowField.rows-1) {
  //    for (int kx = -1; kx <= 1; kx++) {
  //      for (int ky = -1; ky <= 1; ky++) {
  //        if (kx != 0 && ky != 0) {
  //          float angle = PVector.angleBetween(flowField.kernel[kx+1][ky+1], pv);
  //          angle = abs(angle - PI) / PI;
  //          if (angle > 0.5 ) {
  //            pv.mult(angle * angle * 10);
  //            flowField.field[x+kx][y+ky].add(pv);
  //          }
  //        }
  //      }
  //    }
  //  }
  //}

  // Renders a vector object 'v' as an arrow and a position 'x,y'
  void drawVector(PVector v, float x, float y, float scayl) {
    pushMatrix();
    // Translate to position to render vector
    translate(x + resolution * 0.5, y + resolution * 0.5);
    strokeWeight(1);
    // Call vector heading function to get direction (note that pointing up is a heading of 0) and rotate
    rotate(v.heading());
    // Calculate length of vector & scale it to be bigger or smaller if necessary
    float len = max(1, v.mag()*scayl);
    // Draw three lines to make an arrow (draw pointing up since we've rotate to the proper direction)
    line(0, 0, len, 0);
    popMatrix();
  }

  PVector lookup(PVector location) {
    int column = int(constrain(location.x/resolution, 0, cols-1));
    int row = int(constrain(location.y/resolution, 0, rows-1));
    return field[column][row].copy();
  }

  float lookupMag(PVector location) {
    return field[columnForX(location.x)][rowForY(location.y)].mag();
  }

  int columnForX(float x) {
    int column = int(constrain(x/resolution, 0, cols-1));
    return column;
  }

  int rowForY(float y) {
    int row = int(constrain(y/resolution, 0, rows-1));
    return row;
  }

  float getVelocityMagAtLocation(PVector location) {
    return vMags[columnForX(location.x)][rowForY(location.y)];
  }

  float getShapedVelocityMagAtLocation(PVector location) {
    return vMagsShaped[columnForX(location.x)][rowForY(location.y)];
  }
}
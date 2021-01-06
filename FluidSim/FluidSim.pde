// initialize box parameters
float boxLen = 600;

// initialize fluid compuatation parameters
Integer N = 300;
float[] u = new float[(N+2)*(N+2)];
float[] v = new float[(N+2)*(N+2)];
float[] dens = new float[(N+2)*(N+2)];
float[] buffer0 = new float[(N+2)*(N+2)];
float[] buffer1 = new float[(N+2)*(N+2)];
int IX(int i, int j){return (i+(N+2)*j);}
float k_dens = 20, k_vel = 30;

// initialize source parameters
float[] s = new float[(N+2)*(N+2)];
float[] uSrc = new float[(N+2)*(N+2)];
float[] vSrc = new float[(N+2)*(N+2)];

// initialize fluid display parameters
float dLen = boxLen / N;

void setup(){
  size(600, 600);
  background(0);
  
  // water initialization
  for (int i = 0; i < (N+2)*(N+2); i++){
    u[i] = random(1.0)-0.5; v[i] = random(1.0)-0.5;
    //u[i] = 0.0; v[i] = 0.0;
    s[i] = 0.0; uSrc[i] = 0.0; vSrc[i] = 0.0;
    dens[i] = random(1.0)-0.5;
  }
  
  paused = true;
}


void draw(){
  if (!paused)
    update(0.1);
  // camera & light settings
  background(0);
  
  // render water
  noStroke();
  for (int i = 1; i <= N; i++)
    for (int j = 1; j <= N; j++){
      Vec3 mycolor = colorRep(u[IX(i,j)], v[IX(i,j)], 1.0);
      mycolor.normalized();
      mycolor.mul(clamp(dens[IX(i,j)]*100000,0,5000));
      fill(255-mycolor.x, 255-mycolor.y, 255-mycolor.z);
      rect((i-1)*dLen,(j-1)*dLen, dLen, dLen);
    }
 
}

Vec3 colorRep(float uSpeed, float vSpeed, float a){
  Vec3 myColor = new Vec3(0.0, 0.0, 0.0);
  myColor.x = uSpeed*a;
  myColor.y = (-uSpeed*0.5-vSpeed*sqrt(3)*0.5)*a;
  myColor.z = (-uSpeed*0.5+vSpeed*sqrt(3)*0.5)*a;
  return myColor;
}

float max_val;
void update(float dt){
  // update velocities
  velUpdate(dt);
  // update density
  densUpdate(dt);
  // update src
  for (int i = 0; i < (N+2)*(N+2); i++){
    s[i] = s[i]>1.0 ? s[i]-1.0:0.0;
    uSrc[i] = uSrc[i]>1.0 ? uSrc[i]-1.0:0.0;
    vSrc[i] = vSrc[i]>1.0 ? vSrc[i]-1.0:0.0;
  }
}




float leftTime;

int srcX, srcY;
void mousePressed() {
  // store mouse position
  srcX=int(clamp(mouseX/dLen, 0, N+1));
  srcY=int(clamp(mouseY/dLen, 0, N+1));
  if(mouseButton==LEFT)
    if (ctrlPressed)
      s[IX(srcX, srcY)] += 100.0;
    else
      s[IX(srcX, srcY)] -= 100.0;
}

void mouseReleased(){
  if (mouseButton==RIGHT){
  int dirX = int(clamp(mouseX/dLen, 0, N+1));
  int dirY = int(clamp(mouseY/dLen, 0, N+1));
  u[IX(srcX, srcY)] += dirX - srcX;
  v[IX(srcX, srcY)] += dirY - srcY;
  }
}

void mouseDragged(){
   int x = int(clamp(mouseX/dLen, 0, N+1));
   int y = int(clamp(mouseY/dLen, 0, N+1));
   if (mouseButton==LEFT){
     if (ctrlPressed)
      s[IX(x,y)] += 100.0;
     else s[IX(x,y)] -= 100.0;
   }
}

boolean paused, ctrlPressed;
void keyPressed(){
  if (key == ' '){ paused = !paused;}
  if (keyCode == CONTROL){ ctrlPressed = true;}
}

void keyReleased(){
  if (keyCode == CONTROL){ctrlPressed = false;}
}

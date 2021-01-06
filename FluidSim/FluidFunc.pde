void densUpdate(float dt){
  // inject from sources
  addSrc(dens, s, dt);
   
  // add diffusion
  valDup(dens, buffer0);
  diffusion(dens, buffer0, k_dens, dt, 0);
  
  // follow velocity field
  valDup(dens, buffer0);
  advect(dens, buffer0, u, v, dt, 0);
}

void velUpdate(float dt){
  // inject from srcs
  addSrc(u, uSrc, dt);
  addSrc(v, vSrc, dt);
  
  // add diffusion
  valDup(u, buffer0); diffusion(u, buffer0, k_vel, dt, 1);
  valDup(v, buffer0); diffusion(v, buffer0, k_vel, dt, 2);
  
  // follow velocity field
  project(u, v, buffer0, buffer1);
  valDup(u, buffer0);
  valDup(v, buffer1);
  advect(u, buffer0, buffer0, buffer1, dt, 1);
  advect(v, buffer1, buffer0, buffer1, dt, 2);
  project(u, v, buffer0, buffer1);
}

float clamp(float val, float min, float max){
  return max(min, min(max, val));
}

void setBnd(float[] state, int bounceChoice){
  // bouncing boundary condition
  for (int i = 1; i <= N; i++){
    // bounceChoice==1 -> apply bouncing boundary condition in y direction
    state[IX(0, i)] = (bounceChoice==1) ? -state[IX(1, i)]:state[IX(1,i)];
    state[IX(N+1, i)] = (bounceChoice==1) ? -state[IX(N, i)]:state[IX(N, i)];
    // bounceChoice==2 -> apply bouncing boundary condition in x direction
    state[IX(i, 0)] = (bounceChoice==2) ? -state[IX(i, 1)]:state[IX(i, 1)];
    state[IX(i, N+1)] = (bounceChoice==2) ? -state[IX(i, N)]:state[IX(i, N)];
  }
  state[IX(0, 0)] = (state[IX(1, 0)]+state[IX(0, 1)])/2.0;
  state[IX(0, N+1)] = (state[IX(1, N+1)]+state[IX(0, N)])/2.0;
  state[IX(N+1, 0)] = (state[IX(N, 0)]+state[IX(N+1, 1)])/2.0;
  state[IX(N+1, N+1)] = (state[IX(N, N+1)]+state[IX(N+1, N)])/2.0;
}

void addSrc(float[] target, float[] src, float dt){
  for (int i = 0; i < (N+2)*(N+2); i++)
    target[i] += dt*src[i];
}

void diffusion(float[] state, float[] state_prev, float k, float dt, int bouncingChoice){
  int diffIterNum = 20;
  float diffParam = dt*k/dLen/dLen;
  for (int iter = 0; iter < diffIterNum; iter++){
    // gauss-seidel relaxation
    for (int i = 1; i <= N; i++)
      for (int j = 1; j <= N; j++)
        state[IX(i,j)] = (state_prev[IX(i,j)]
                         +diffParam*(state[IX(i-1, j)]+state[IX(i+1, j)]+state[IX(i, j-1)]+state[IX(i, j+1)])) / (1.0+4*diffParam);
    // set boundary
    setBnd(state, bouncingChoice);
  }
}

void advect(float[] state, float[] state_prev, float[] u_speed, float[] v_speed, float dt, int bouncingChoice){
  for (int i = 1; i <= N; i++)
    for (int j = 1; j <= N; j++){
      // backtrace
      float i_prev = clamp(i - dt*u_speed[IX(i,j)]/dLen, 0.5, N+0.5);
      float j_prev = clamp(j - dt*v_speed[IX(i,j)]/dLen, 0.5, N+0.5);
      // add velocity field impact
      int i_left = (int)i_prev, j_left = (int)j_prev;
      int i_right = i_left+1, j_right = j_left+1;
      state[IX(i,j)] = (i_prev-i_left)*(j_prev-j_left)*state_prev[IX(i_left, j_left)]
                      +(i_prev-i_left)*(j_right-j_prev)*state_prev[IX(i_left, j_right)]
                      +(i_right-i_prev)*(j_prev-j_left)*state_prev[IX(i_right, j_left)]
                      +(i_right-i_prev)*(j_right-j_prev)*state_prev[IX(i_right, j_right)];
    }
  // set boundary
  setBnd(state, bouncingChoice);
}

void valDup(float[] state, float[] state_prev){
  //float x;
  for (int i = 0; i < (N+2)*(N+2); i++){
    //x=state_prev[i];
    state_prev[i] = state[i];
    //state[i]=x;
  }
}

void project(float[] u_speed, float[] v_speed, float[] p, float[] div){ 
  for (int i = 1; i <= N; i++)
    for (int j = 1; j <= N; j++){
      div[IX(i,j)] = -0.5*dLen*(u_speed[IX(i+1, j)] - u_speed[IX(i-1, j)]
                               +v_speed[IX(i, j+1)] - v_speed[IX(i, j-1)]);
      p[IX(i,j)] = 0;
    }
  setBnd(div, 0); setBnd(p, 0);
  
  int diffIterNum = 10;
  for (int iter = 0; iter < diffIterNum; iter++){
    // gauss-seidel relaxation
    for (int i = 1; i <= N; i++)
      for (int j = 1; j <= N; j++)
        p[IX(i,j)] = (div[IX(i,j)]
                     +(p[IX(i-1, j)]+p[IX(i+1, j)]+p[IX(i, j-1)]+p[IX(i, j+1)])) / 4;
    // set boundary
    setBnd(p, 0);
  }
  
  // subtract gradient field
  for (int i = 1; i <= N; i++)
    for (int j = 1; j <= N; j++){
      u_speed[IX(i, j)] -= (p[IX(i+1, j)]-p[IX(i-1, j)])/dLen/2.0;
      v_speed[IX(i, j)] -= (p[IX(i, j+1)]-p[IX(i, j-1)])/dLen/2.0;
    }
  setBnd(u_speed, 1); setBnd(v_speed, 2);
}

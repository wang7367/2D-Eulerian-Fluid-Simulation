//Vector Library [2D]
//CSCI 5611 Vector 3 Library [Incomplete]

//Instructions: Add 3D versions of all of the 2D vector functions
//              Vec3 must also support the cross product.
public class Vec3 {
  public float x, y, z;
  
  public Vec3(float x, float y, float z){
    this.x = x;
    this.y = y;
    this.z = z;
  }
  
  public String toString(){
    return "(" + x+ ", " + y + ", " + z + ")";
  }
  
  public float length(){
    return sqrt(x*x+y*y+z*z);
  }
  
  public Vec3 plus(Vec3 rhs){
    return new Vec3(x+rhs.x,y+rhs.y,z+rhs.z);
  }
  
  public void add(Vec3 rhs){
    x += rhs.x;
    y += rhs.y;
    z += rhs.z;
  }
  
  public Vec3 minus(Vec3 rhs){
    return new Vec3(x-rhs.x,y-rhs.y,z-rhs.z);
  }
  
  public void subtract(Vec3 rhs){
    x -= rhs.x;
    y -= rhs.y;
    z -= rhs.z;
  }
  
  public Vec3 times(float rhs){
    return new Vec3(rhs*x, rhs*y, rhs*z);
  }
  
  public void mul(float rhs){
    x *= rhs;
    y *= rhs;
    z *= rhs;
  }
  
  public void normalize(){
    float vecLen = this.length();
    x /= vecLen;
    y /= vecLen;
    z /= vecLen;
  }
  
  public Vec3 normalized(){
    float vecLen = this.length();
    return new Vec3(x/vecLen,y/vecLen,z/vecLen);
  }
  
  public float distanceTo(Vec3 rhs){
    float dx = x - rhs.x;
    float dy = y - rhs.y;
    float dz = z - rhs.z;
    return sqrt(dx*dx + dy*dy + dz*dz);
  }
}

Vec3 interpolate(Vec3 a, Vec3 b, float t){
  return a.plus(b.minus(a).times(t)); 
}

float dot(Vec3 a, Vec3 b){
  return a.x*b.x + a.y*b.y + a.z*b.z;
}

Vec3 cross(Vec3 a, Vec3 b){
  return new Vec3(a.y*b.z-b.y*a.z,
                  b.x*a.z-b.z*a.x,
                  a.x*b.y-b.x*a.y);
}

Vec3 projAB(Vec3 a, Vec3 b){
  Vec3 bnorm = b.normalized();
  return bnorm.times(dot(a, bnorm));
}

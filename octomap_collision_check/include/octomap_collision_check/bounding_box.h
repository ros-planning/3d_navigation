#ifndef OBJECT_MAPPING_BOUNDING_BOX_H
#define OBJECT_MAPPING_BOUNDING_BOX_H

#include <Eigen/Geometry>

namespace omapping {
   
  class AABB {
    
  public:

    typedef Eigen::Vector3f Point;
    
  public:

  /* AABB(const Point& center_, const Point& bounds_) */
  /*   : center(center_), bounds(bounds_) {} */

  AABB() 
    : center (0,0,0), bounds(1,1,1) {
  }


  AABB(const Point& min, const Point& max) {
    bounds = (max - min) / 2.;
    center = min + bounds;
  }
  
  public:
        
    bool containsPoint(const Point& p) const {
      Point delta = center - p;
     
      if ( (abs(delta(0)) <= bounds(0)) &&
           (abs(delta(1)) <= bounds(1)) &&
           (abs(delta(2)) <= bounds(2)) 
           ) return true;
      else return false;
    }

    int intersects(const AABB& other) const {
      Point delta;
      delta(0) = fabs(this->center(0) - other.center(0));
      delta(1) = fabs(this->center(1) - other.center(1));
      delta(2) = fabs(this->center(2) - other.center(2));

      if((delta(0) > this->bounds(0) + other.bounds(0)) ||
         (delta(1) > this->bounds(1) + other.bounds(1)) ||
         (delta(2) > this->bounds(2) + other.bounds(2))
         ) return 0;  // no intersection
                
      else if((delta(0) + other.bounds(0) <= this->bounds(0)) &&
              (delta(1) + other.bounds(1) <= this->bounds(1)) &&
              (delta(2) + other.bounds(2) <= this->bounds(2))
              ) return 1; // this contains other
                
      else if((delta(0) + this->bounds(0) <= other.bounds(0)) &&
              (delta(1) + this->bounds(1) <= other.bounds(1)) &&
              (delta(2) + this->bounds(2) <= other.bounds(2))
              ) return 2; // this is contained in other
                
      else return 3; // partial intersection 
    }

  public:
    
    Point center;
    Point bounds;
        
  };


  inline std::ostream& operator<< (std::ostream& s, const AABB& b) {
    s << "center: <" << b.center(0) << " , " << b.center(1) << " , " << b.center(2)
      <<">\nbounds:  " << b.bounds(0) << " , " << b.bounds(1) << " , " << b.bounds(2);
    return s;
  }


  class OBB {

  public:

    typedef Eigen::Vector3f Point;
    typedef Eigen::Matrix3f Rotation;
    typedef Eigen::Affine3f Transform;
      
  public:
    
  OBB() : center(0,0,0), rotation(Eigen::Matrix3f::Identity()), bounds(1,1,1) { }
    
  OBB(const Point& center_, const Eigen::Matrix3f& rotation_, const Point& bounds_)
    : center(center_), rotation(rotation_), bounds(bounds_) { }

    OBB(const Point& center_, float roll, float pitch, float yaw, const Point& bounds_) {
      float A=cosf(yaw),  B=sinf(yaw),  C=cosf(pitch), D=sinf(pitch),
        E=cosf(roll), F=sinf(roll), DE=D*E,        DF=D*F;
      rotation(0,0) = A*C;  rotation(0,1) = A*DF - B*E;  rotation(0,2) = B*F + A*DE;
      rotation(1,0) = B*C;  rotation(1,1) = A*E + B*DF;  rotation(1,2) = B*DE - A*F;
      rotation(2,0) = -D;   rotation(2,1) = C*F;         rotation(2,2) = C*E;
      center = center_;
      bounds = bounds_;
    }
    
  public:


    Point transformToOBB(const Point& p) const {

      Eigen::Translation3f translation (center);
      Transform t;
      t = translation * rotation;

      Point retval = t.inverse() * p;
      return retval;
    }


    Point transformToWorld(const Point& p) const {      
      Point retval = rotation * p + center;
      return retval;
    }

    
    bool containsPoint(const Point& point) const {
      Point transformed = transformToOBB(point);
      return this->toAABB().containsPoint(transformed);
    }


    bool intersects(const AABB& other) {
      OBB other_obb (other.center, 0,0,0, other.bounds);
      return this->intersects(other_obb);
    }


    bool intersects(const OBB& other) {

      Point ax (1,0,0);
      Point ay (0,1,0);
      Point az (0,0,1);

      // this
      Point a = this->bounds; //extents
      Point Pa = this->center; //position
      Point A[3]; //orthonormal basis       
      A[0] = this->rotation * ax;
      A[1] = this->rotation * ay;
      A[2] = this->rotation * az;
      
      /* std::cout << "a: " << a << std::endl;  */
      /* std::cout << "Pa: " << Pa << std::endl;  */
      /* for (int i=0;i<3;i++) { */
      /*   std::cout << "A[" << i<< "]:\n" << A[i] << std::endl; */
      /* } */
      
      // other
      Point b = other.bounds; //extents
      Point Pb = other.center; //position
      Point B[3]; //orthonormal basis
      B[0] = other.rotation * ax;
      B[1] = other.rotation * ay;
      B[2] = other.rotation * az;

      /* std::cout << "b: " << b << std::endl;  */
      /* std::cout << "Pb: " << Pb << std::endl;  */
      /* for (int i=0;i<3;i++) { */
      /*   std::cout << "B[" << i<< "]:\n" << B[i] << std::endl; */
      /* } */

      return OBBOverlap(a,Pa,A, b,Pb,B);
    }

    // untested
    OBB transformOBB(Transform& t) const {
      return OBB(t * center, t.rotation() * rotation, bounds);
    }


  protected:

    // returns self-centered AABB
    AABB toAABB() const {
      Point min (-bounds(0), -bounds(1), -bounds(2)); 
      Point max (+bounds(0), +bounds(1), +bounds(2)); 
      return AABB(min, max);
    }
        

    /*
      code from:
      "Simple Intersection Tests For Games", by Miguel Gomez:
      http://www.gamasutra.com/view/feature/3383/simple_intersection_tests_for_games.php?page=5
    */

    bool OBBOverlap (
                     //A
                     Point& a, //extents
                     Point& Pa, //position
                     Point* A, //orthonormal basis 
                     //B
                     Point& b, //extents
                     Point& Pb, //position
                     Point* B //orthonormal basis
                     )  {

      //translation, in parent frame
      Point v = Pb - Pa; 

      //translation, in A's frame
      Point T( v.dot(A[0]), v.dot(A[1]), v.dot(A[2]) );

      //B's basis with respect to A's local frame
      double R[3][3];
      float ra, rb, t;
      long i, k;

      //calculate rotation matrix
      for( i=0 ; i<3 ; i++ )
        for( k=0 ; k<3 ; k++ ) 
          R[i][k] = A[i].dot(B[k]); 

      /*ALGORITHM: Use the separating axis test for all 15 potential
        separating axes. If a separating axis could not be found, the two
        boxes overlap. */

      //A's basis vectors
      for( i=0 ; i<3 ; i++ ) {
          
        ra = a[i];

        rb =
          b[0]*fabs(R[i][0]) + b[1]*fabs(R[i][1]) + b[2]*fabs(R[i][2]);

        t = fabs( T[i] );

        if( t > ra + rb )
          return false;
      }

      //B's basis vectors
      for( k=0 ; k<3 ; k++ ) {
        ra =
          a[0]*fabs(R[0][k]) + a[1]*fabs(R[1][k]) + a[2]*fabs(R[2][k]); 

        rb = b[k];

        t =
          fabs( T[0]*R[0][k] + T[1]*R[1][k] +
                T[2]*R[2][k] );

        if( t > ra + rb )
          return false;
      }

      //9 cross products

      //L = A0 x B0
      ra =
        a[1]*fabs(R[2][0]) + a[2]*fabs(R[1][0]);

      rb =
        b[1]*fabs(R[0][2]) + b[2]*fabs(R[0][1]);

      t =
        fabs( T[2]*R[1][0] -
              T[1]*R[2][0] );

      if( t > ra + rb )
        return false;

      //L = A0 x B1
      ra =
        a[1]*fabs(R[2][1]) + a[2]*fabs(R[1][1]);

      rb =
        b[0]*fabs(R[0][2]) + b[2]*fabs(R[0][0]);

      t =
        fabs( T[2]*R[1][1] -
              T[1]*R[2][1] );

      if( t > ra + rb )
        return false;

      //L = A0 x B2
      ra =
        a[1]*fabs(R[2][2]) + a[2]*fabs(R[1][2]);

      rb =
        b[0]*fabs(R[0][1]) + b[1]*fabs(R[0][0]);

      t =
        fabs( T[2]*R[1][2] -
              T[1]*R[2][2] );

      if( t > ra + rb )
        return false;

      //L = A1 x B0
      ra =
        a[0]*fabs(R[2][0]) + a[2]*fabs(R[0][0]);

      rb =
        b[1]*fabs(R[1][2]) + b[2]*fabs(R[1][1]);

      t =
        fabs( T[0]*R[2][0] -
              T[2]*R[0][0] );

      if( t > ra + rb )
        return false;

      //L = A1 x B1
      ra =
        a[0]*fabs(R[2][1]) + a[2]*fabs(R[0][1]);

      rb =
        b[0]*fabs(R[1][2]) + b[2]*fabs(R[1][0]);

      t =
        fabs( T[0]*R[2][1] -
              T[2]*R[0][1] );

      if( t > ra + rb )
        return false;

      //L = A1 x B2
      ra =
        a[0]*fabs(R[2][2]) + a[2]*fabs(R[0][2]);

      rb =
        b[0]*fabs(R[1][1]) + b[1]*fabs(R[1][0]);

      t =
        fabs( T[0]*R[2][2] -
              T[2]*R[0][2] );

      if( t > ra + rb )
        return false;

      //L = A2 x B0
      ra =
        a[0]*fabs(R[1][0]) + a[1]*fabs(R[0][0]);

      rb =
        b[1]*fabs(R[2][2]) + b[2]*fabs(R[2][1]);

      t =
        fabs( T[1]*R[0][0] -
              T[0]*R[1][0] );

      if( t > ra + rb )
        return false;

      //L = A2 x B1
      ra =
        a[0]*fabs(R[1][1]) + a[1]*fabs(R[0][1]);

      rb =
        b[0] *fabs(R[2][2]) + b[2]*fabs(R[2][0]);

      t =
        fabs( T[1]*R[0][1] -
              T[0]*R[1][1] );

      if( t > ra + rb )
        return false;

      //L = A2 x B2
      ra =
        a[0]*fabs(R[1][2]) + a[1]*fabs(R[0][2]);

      rb =
        b[0]*fabs(R[2][1]) + b[1]*fabs(R[2][0]);

      t =
        fabs( T[1]*R[0][2] -
              T[0]*R[1][2] );

      if( t > ra + rb )
        return false;

      /*no separating axis found,
        the two boxes overlap */

      return true;
    }

  public:
    
    Point center;
    Rotation rotation;
    Point bounds;
  
  };

  inline std::ostream& operator<< (std::ostream& s, const OBB& b) {
    s << "center: <" << b.center(0) << " , " << b.center(1) << " , " << b.center(2)
      <<">\nbounds:  " << b.bounds(0) << " , " << b.bounds(1) << " , " << b.bounds(2);
    return s;
  }

} // namespace 

#endif 

#ifndef DEPTH_ODOMETRY_ITERATIVE_CLOSEST_SURFACE_H
#define DEPTH_ODOMETRY_ITERATIVE_CLOSEST_SURFACE_H

namespace depth_odometry {

class SurfaceMatcher {
  public:
    SurfaceMatcher();
    ~SurfaceMatcher();
    void setSourceAndTarget();
    void update();
  private:

}; // class SurfaceMatcher

} // namespace depth_odometry


#endif

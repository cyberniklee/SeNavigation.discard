#ifndef _DRAW_INTERFACE_H_
#define _DRAW_INTERFACE_H_

class DrawInterface
{
public:
  virtual void
  drawPoint (const Eigen::Vector2f& pointWorldFrame) = 0;
  virtual void
  drawArrow (const Eigen::Vector3f& poseWorld) = 0;
  virtual void
  drawCovariance (const Eigen::Vector2f& mean, const Eigen::Matrix2f& cov) = 0;

  virtual void
  setScale (double scale) = 0;
  virtual void
  setColor (double r, double g, double b, double a = 1.0) = 0;

  virtual void
  sendAndResetData () = 0;
};

#endif

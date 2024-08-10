#include "icp_graph.hpp"


ICPGraph::ICPGraph(const Eigen::Vector3d& point) : _point(point) {};

void ICPGraph::computeError() {
  const g2o::VertexSE3Expmap* pose = static_cast<const g2o::VertexSE3Expmap*> (_vertices[0]);
  _error = _measurement - pose->estimate().map(_point);
}

void ICPGraph::linearizeOplus() {
  g2o::VertexSE3Expmap* pose = static_cast<g2o::VertexSE3Expmap*>(_vertices[0]);
  g2o::SE3Quat T(pose->estimate());
  Eigen::Vector3d xyz_trans = T.map(_point);
  double x = xyz_trans[0];
  double y = xyz_trans[1];
  double z = xyz_trans[2];

  _jacobianOplusXi(0,0) = 0;
  _jacobianOplusXi(0,1) = -z;
  _jacobianOplusXi(0,2) = y;
  _jacobianOplusXi(0,3) = -1;
  _jacobianOplusXi(0,4) = 0;
  _jacobianOplusXi(0,5) = 0;

  _jacobianOplusXi(1,0) = z;
  _jacobianOplusXi(1,1) = 0;
  _jacobianOplusXi(1,2) = -x;
  _jacobianOplusXi(1,3) = 0;
  _jacobianOplusXi(1,4) = -1;
  _jacobianOplusXi(1,5) = 0;

  _jacobianOplusXi(2,0) = -y;
  _jacobianOplusXi(2,1) = x;
  _jacobianOplusXi(2,2) = 0;
  _jacobianOplusXi(2,3) = 0;
  _jacobianOplusXi(2,4) = 0;
  _jacobianOplusXi(2,5) = -1;
}

bool ICPGraph::read(std::istream& in) {}
bool ICPGraph::write(std::ostream& out) const {}
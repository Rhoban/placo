#include "placo/model/algorithms.h"

namespace placo::model
{
Data::Data(const Model& model)
{
  liMi.assign(model.njoints, Eigen::Affine3d::Identity());
  oMi.assign(model.njoints, Eigen::Affine3d::Identity());
  oMf.assign(model.frames.size(), Eigen::Affine3d::Identity());
  v.assign(model.njoints, Vector6d::Zero());
  ov.assign(model.njoints, Vector6d::Zero());
  a.assign(model.njoints, Vector6d::Zero());
  f.assign(model.njoints, Vector6d::Zero());
  J = Matrix6Xd::Zero(6, model.nv);
  dJ = Matrix6Xd::Zero(6, model.nv);
  M = Eigen::MatrixXd::Zero(model.nv, model.nv);
  nle = Eigen::VectorXd::Zero(model.nv);
  g = Eigen::VectorXd::Zero(model.nv);
  com.setZero();
  subtree_mass.assign(model.njoints, 0.);
  subtree_com.assign(model.njoints, Eigen::Vector3d::Zero());
  subtree_momentum.assign(model.njoints, Eigen::Vector3d::Zero());
  oYcrb.assign(model.njoints, Matrix6d::Zero());
}

void forward_kinematics(const Model& model, Data& data, const Eigen::VectorXd& q)
{
  for (int i = 1; i < model.njoints; i++)
  {
    data.liMi[i] = model.joint_placements[i] * model.joints[i].transform(q);
    data.oMi[i] = data.oMi[model.parents[i]] * data.liMi[i];
  }
}

void compute_joint_jacobians(const Model& model, Data& data, const Eigen::VectorXd& q)
{
  forward_kinematics(model, data, q);

  for (int i = 1; i < model.njoints; i++)
  {
    const Joint& joint = model.joints[i];
    const Matrix6Xd& S = joint.motion_subspace();
    for (int k = 0; k < joint.nv; k++)
    {
      data.J.col(joint.idx_v + k) = act(data.oMi[i], S.col(k));
    }
  }
}

void compute_joint_jacobians_time_variation(const Model& model, Data& data, const Eigen::VectorXd& q,
                                            const Eigen::VectorXd& qd)
{
  forward_kinematics(model, data, q);

  for (int i = 1; i < model.njoints; i++)
  {
    const Joint& joint = model.joints[i];
    const Matrix6Xd& S = joint.motion_subspace();

    // Joint velocity (local frame), propagated from the parent
    data.v[i] = S * qd.segment(joint.idx_v, joint.nv);
    if (model.parents[i] > 0)
    {
      data.v[i] += act_inv(data.liMi[i], data.v[model.parents[i]]);
    }
    data.ov[i] = act(data.oMi[i], data.v[i]);

    for (int k = 0; k < joint.nv; k++)
    {
      data.J.col(joint.idx_v + k) = act(data.oMi[i], S.col(k));
      data.dJ.col(joint.idx_v + k) = motion_cross(data.ov[i], data.J.col(joint.idx_v + k));
    }
  }
}

void update_frame_placements(const Model& model, Data& data)
{
  for (size_t k = 0; k < model.frames.size(); k++)
  {
    const Frame& frame = model.frames[k];
    data.oMf[k] = data.oMi[frame.parent_joint] * frame.placement;
  }
}

void point_jacobian(const Model& model, const Data& data, int joint, const std::vector<int>& support,
                    const Eigen::Affine3d& T_world_point, ReferenceFrame ref, const std::vector<int>& columns,
                    Eigen::MatrixXd& J, bool time_variation)
{
  // The joint Jacobians are the spatial velocities of the world origin, in world axes, for each degree of freedom
  // of the support (the other columns are zero)
  const Matrix6Xd& J_world = time_variation ? data.dJ : data.J;
  const Eigen::Vector3d& p = T_world_point.translation();
  const Eigen::Matrix3d R = T_world_point.linear();

  // Velocity of the point (world axes), and in the point frame
  Vector6d v_point_local = Vector6d::Zero();
  Eigen::Vector3d v_point_world = Eigen::Vector3d::Zero();
  if (time_variation)
  {
    const Vector6d& ov_joint = data.ov[joint];
    v_point_world = ov_joint.head<3>() + ov_joint.tail<3>().cross(p);
    v_point_local = act_inv(T_world_point, ov_joint);
  }

  J.resize(6, columns.size());
  int k_support = 0;
  for (int k = 0; k < (int)columns.size(); k++)
  {
    int column = columns[k];
    while (k_support < (int)support.size() && support[k_support] < column)
    {
      k_support += 1;
    }
    if (k_support == (int)support.size() || support[k_support] != column)
    {
      J.col(k).setZero();
      continue;
    }

    Eigen::Vector3d linear = J_world.col(column).head<3>();
    Eigen::Vector3d angular = J_world.col(column).tail<3>();

    if (ref == LOCAL_WORLD_ALIGNED)
    {
      // Velocity of the point
      linear += angular.cross(p);
      if (time_variation)
      {
        linear -= v_point_world.cross(data.J.col(column).tail<3>());
      }
    }
    else if (ref == LOCAL)
    {
      linear = R.transpose() * (linear + angular.cross(p));
      angular = R.transpose() * angular;
      if (time_variation)
      {
        Vector6d correction = motion_cross(v_point_local, act_inv(T_world_point, data.J.col(column)));
        linear -= correction.head<3>();
        angular -= correction.tail<3>();
      }
    }

    J.col(k).head<3>() = linear;
    J.col(k).tail<3>() = angular;
  }
}

void crba(const Model& model, Data& data, const Eigen::VectorXd& q)
{
  compute_joint_jacobians(model, data, q);

  for (int i = 1; i < model.njoints; i++)
  {
    data.oYcrb[i] = model.inertias[i].transformed(data.oMi[i]).matrix();
  }

  // Backward pass: the composite inertia of each subtree gives the mass matrix entries between its joint and the
  // joints of its support
  for (int i = model.njoints - 1; i > 0; i--)
  {
    const Joint& joint = model.joints[i];
    for (int k = 0; k < joint.nv; k++)
    {
      const int column = joint.idx_v + k;
      const Vector6d F = data.oYcrb[i] * data.J.col(column);

      for (int ancestor : model.supports[i])
      {
        const Joint& ancestor_joint = model.joints[ancestor];
        for (int l = 0; l < ancestor_joint.nv; l++)
        {
          const int row = ancestor_joint.idx_v + l;
          data.M(row, column) = data.J.col(row).dot(F);
          data.M(column, row) = data.M(row, column);
        }
      }
    }

    if (model.parents[i] > 0)
    {
      data.oYcrb[model.parents[i]] += data.oYcrb[i];
    }
  }

  data.M.diagonal() += model.armature;
}

Eigen::VectorXd rnea(const Model& model, Data& data, const Eigen::VectorXd& q, const Eigen::VectorXd& qd,
                     const Eigen::VectorXd& qdd)
{
  // Forward pass, in the joint frames. The gravity is taken into account with a base acceleration
  Vector6d a0 = Vector6d::Zero();
  a0.head<3>() = -model.gravity;

  for (int i = 1; i < model.njoints; i++)
  {
    const Joint& joint = model.joints[i];
    const Matrix6Xd& S = joint.motion_subspace();
    const int parent = model.parents[i];

    data.liMi[i] = model.joint_placements[i] * joint.transform(q);
    const Vector6d vJ = S * qd.segment(joint.idx_v, joint.nv);

    data.v[i] = vJ;
    if (parent > 0)
    {
      data.v[i] += act_inv(data.liMi[i], data.v[parent]);
    }
    data.a[i] = S * qdd.segment(joint.idx_v, joint.nv) + motion_cross(data.v[i], vJ) +
                act_inv(data.liMi[i], parent > 0 ? data.a[parent] : a0);

    const Inertia& inertia = model.inertias[i];
    data.f[i] = inertia.momentum(data.a[i]) + force_cross(data.v[i], inertia.momentum(data.v[i]));
  }

  // Backward pass
  Eigen::VectorXd tau(model.nv);
  for (int i = model.njoints - 1; i > 0; i--)
  {
    const Joint& joint = model.joints[i];
    tau.segment(joint.idx_v, joint.nv) = joint.motion_subspace().transpose() * data.f[i];

    const int parent = model.parents[i];
    if (parent > 0)
    {
      data.f[parent] += act_force(data.liMi[i], data.f[i]);
    }
  }
  tau.array() += model.armature.array() * qdd.array();

  return tau;
}

void non_linear_effects(const Model& model, Data& data, const Eigen::VectorXd& q, const Eigen::VectorXd& qd)
{
  data.nle = rnea(model, data, q, qd, Eigen::VectorXd::Zero(model.nv));
}

void generalized_gravity(const Model& model, Data& data, const Eigen::VectorXd& q)
{
  data.g = rnea(model, data, q, Eigen::VectorXd::Zero(model.nv), Eigen::VectorXd::Zero(model.nv));
}

void center_of_mass(const Model& model, Data& data)
{
  data.subtree_mass[0] = 0.;
  data.subtree_com[0].setZero();
  for (int i = 1; i < model.njoints; i++)
  {
    const Inertia& inertia = model.inertias[i];
    data.subtree_mass[i] = inertia.mass;
    data.subtree_com[i] = inertia.mass * (data.oMi[i] * inertia.lever);
  }

  for (int i = model.njoints - 1; i > 0; i--)
  {
    const int parent = model.parents[i];
    data.subtree_mass[parent] += data.subtree_mass[i];
    data.subtree_com[parent] += data.subtree_com[i];
  }

  for (int i = 0; i < model.njoints; i++)
  {
    if (data.subtree_mass[i] > 0.)
    {
      data.subtree_com[i] /= data.subtree_mass[i];
    }
    else
    {
      data.subtree_com[i] = data.oMi[i].translation();
    }
  }

  data.com = data.subtree_com[0];
}

Eigen::Matrix3Xd com_jacobian(const Model& model, Data& data, const Eigen::VectorXd& q)
{
  compute_joint_jacobians(model, data, q);
  center_of_mass(model, data);

  // Each degree of freedom moves the center of mass of the subtree of its joint
  Eigen::Matrix3Xd Jcom(3, model.nv);
  const double total_mass = data.subtree_mass[0];
  for (int i = 1; i < model.njoints; i++)
  {
    const Joint& joint = model.joints[i];
    for (int k = 0; k < joint.nv; k++)
    {
      const int column = joint.idx_v + k;
      Jcom.col(column) = data.subtree_mass[i] *
                         (data.J.col(column).head<3>() + data.J.col(column).tail<3>().cross(data.subtree_com[i])) /
                         total_mass;
    }
  }

  return Jcom;
}

Eigen::Matrix3Xd com_jacobian_time_variation(const Model& model, Data& data, const Eigen::VectorXd& q,
                                             const Eigen::VectorXd& qd)
{
  compute_joint_jacobians_time_variation(model, data, q, qd);
  center_of_mass(model, data);

  // Linear momentum of the subtrees
  for (int i = 1; i < model.njoints; i++)
  {
    const Inertia& inertia = model.inertias[i];
    const Eigen::Vector3d com_i = data.oMi[i] * inertia.lever;
    data.subtree_momentum[i] = inertia.mass * (data.ov[i].head<3>() + data.ov[i].tail<3>().cross(com_i));
  }
  for (int i = model.njoints - 1; i > 0; i--)
  {
    const int parent = model.parents[i];
    if (parent > 0)
    {
      data.subtree_momentum[parent] += data.subtree_momentum[i];
    }
  }

  // Time derivative of the Jacobian of the center of mass of the subtree of each joint
  Eigen::Matrix3Xd dJcom(3, model.nv);
  const double total_mass = data.subtree_mass[0];
  for (int i = 1; i < model.njoints; i++)
  {
    const Joint& joint = model.joints[i];
    for (int k = 0; k < joint.nv; k++)
    {
      const int column = joint.idx_v + k;
      dJcom.col(column) = (data.subtree_mass[i] * (data.dJ.col(column).head<3>() +
                                                   data.dJ.col(column).tail<3>().cross(data.subtree_com[i])) +
                           data.J.col(column).tail<3>().cross(data.subtree_momentum[i])) /
                          total_mass;
    }
  }

  return dJcom;
}

Eigen::MatrixXd centroidal_map(const Model& model, Data& data, const Eigen::VectorXd& q)
{
  compute_joint_jacobians(model, data, q);
  center_of_mass(model, data);

  for (int i = 1; i < model.njoints; i++)
  {
    data.oYcrb[i] = model.inertias[i].transformed(data.oMi[i]).matrix();
  }
  for (int i = model.njoints - 1; i > 0; i--)
  {
    if (model.parents[i] > 0)
    {
      data.oYcrb[model.parents[i]] += data.oYcrb[i];
    }
  }

  // Momentum of the subtree of each joint, expressed at the world origin, then at the center of mass
  Eigen::MatrixXd Ag(6, model.nv);
  for (int i = 1; i < model.njoints; i++)
  {
    const Joint& joint = model.joints[i];
    for (int k = 0; k < joint.nv; k++)
    {
      const int column = joint.idx_v + k;
      Vector6d momentum = data.oYcrb[i] * data.J.col(column);
      momentum.tail<3>() -= data.com.cross(momentum.head<3>());
      Ag.col(column) = momentum;
    }
  }

  return Ag;
}

Eigen::MatrixXd frame_hessian(const Model& model, const Data& data, int frame, int dof)
{
  const int joint = model.frames[frame].parent_joint;
  const Eigen::Affine3d oMf = data.oMi[joint] * model.frames[frame].placement;
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(6, model.nv);

  // Joint of the degree of freedom, which must be in the support of the frame joint and not attached to the
  // universe
  int dof_joint = -1;
  for (int ancestor : model.supports[joint])
  {
    const Joint& ancestor_joint = model.joints[ancestor];
    if (dof >= ancestor_joint.idx_v && dof < ancestor_joint.idx_v + ancestor_joint.nv && ancestor > 0)
    {
      dof_joint = ancestor;
    }
  }
  if (dof_joint < 0 || model.parents[dof_joint] == 0)
  {
    return H;
  }

  // The Jacobian of the degrees of freedom after dof (in the support) varies with dof; the derivatives are
  // expressed in the frame
  for (int ancestor : model.supports[joint])
  {
    const Joint& ancestor_joint = model.joints[ancestor];
    for (int k = 0; k < ancestor_joint.nv; k++)
    {
      const int column = ancestor_joint.idx_v + k;
      if (column < dof || (column > dof && ancestor == dof_joint))
      {
        H.col(column) = act_inv(oMf, motion_cross(data.J.col(column), data.J.col(dof)));
      }
    }
  }

  return H;
}
}  // namespace placo::model

# Pinch gripper visual meshes

`visual/ee_base_link.dae` includes the B0495 camera visual in the gripper base frame.
The two camera geometry groups were extracted from
`openarm_v2_urdf_additional/end-efector_follower_4310_v1.0_urdf_with_cam_view.SLDASM/meshes/ee_base_link_ee_base_link.dae`.
The source 3DXML identifies them as `B0495_simplified` (`TessPart_100.3DRep`): 116 PCB triangles and 2,218 body triangles.

The exported coordinates, normals and colors are retained in meters. The existing gripper geometry is unchanged.
`camera_view_frame` remains an empty reference frame. This update adds visual geometry only; collision and inertial parameters retain their existing definitions.

- Source DAE SHA-256: `5ae94a1e93a9e076dcdbdce11cc95f2d51409b8ce55946a665b40d94e02e8ee2`
- Source 3DXML SHA-256: `f291f865f2678565bc766df211c7dc543c0380f5d5afc24699b84ff5acc81579`

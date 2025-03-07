import numpy as np

import genesis as gs

########################## init ##########################
gs.init(backend=gs.gpu)

########################## create a scene ##########################
scene = gs.Scene(
    viewer_options=gs.options.ViewerOptions(
        camera_pos=(0, -3.5, 2.5),
        camera_lookat=(0.0, 0.0, 0.5),
        camera_fov=30,
        max_FPS=60,
    ),
    sim_options=gs.options.SimOptions(
        dt=0.01,
    ),
    show_viewer=True,
)

########################## entities ##########################
plane = scene.add_entity(
    gs.morphs.Plane(),
)
franka = scene.add_entity(
    gs.morphs.URDF(
        file="go2w_genesis/urdf/go2w_front_description.urdf",
        pos= [0, 0, 0.5]
    ),
)
########################## build ##########################
scene.build()

jnt_names = [
    "FR_hip_joint",
    "FR_thigh_joint",
    "FR_calf_joint",
    "FL_hip_joint",
    "FL_thigh_joint",
    "FL_calf_joint",
    "RR_hip_joint",
    "RR_thigh_joint",
    "RR_calf_joint",
    "RL_hip_joint",
    "RL_thigh_joint",
    "RL_calf_joint",
    # "FL_foot_joint",
    # "FR_foot_joint",
    # "RL_foot_joint",
    # "RR_foot_joint",
]
dofs_idx = [franka.get_joint(name).dof_idx_local for name in jnt_names]

############ Optional: set control gains ############
# set positional gains

# Hard reset
# for i in range(150):
#     if i < 50:
#         franka.set_dofs_position(np.array([1, 1, 0, 0, 0, 0, 0, 0.04, 0.04]), dofs_idx)
#     elif i < 100:
#         franka.set_dofs_position(np.array([-1, 0.8, 1, -2, 1, 0.5, -0.5, 0.04, 0.04]), dofs_idx)
#     else:
#         franka.set_dofs_position(np.array([0, 0, 0, 0, 0, 0, 0, 0, 0]), dofs_idx)

#     scene.step()

# # PD control
for i in range(1250):
#     if i == 0:
#         franka.control_dofs_position(
#             np.array([1, 1, 0, 0, 0, 0, 0, 0.04, 0.04]),
#             dofs_idx,
#         )
#     elif i == 250:
#         franka.control_dofs_position(
#             np.array([-1, 0.8, 1, -2, 1, 0.5, -0.5, 0.04, 0.04]),
#             dofs_idx,
#         )
#     elif i == 500:
#         franka.control_dofs_position(
#             np.array([0, 0, 0, 0, 0, 0, 0, 0, 0]),
#             dofs_idx,
#         )
#     elif i == 750:
#         # control first dof with velocity, and the rest with position
#         franka.control_dofs_position(
#             np.array([0, 0, 0, 0, 0, 0, 0, 0, 0])[1:],
#             dofs_idx[1:],
#         )
#         franka.control_dofs_velocity(
#             np.array([1.0, 0, 0, 0, 0, 0, 0, 0, 0])[:1],
#             dofs_idx[:1],
#         )
#     elif i == 1000:
#         franka.control_dofs_force(
#             np.array([0, 0, 0, 0, 0, 0, 0, 0, 0]),
#             dofs_idx,
#         )
#     # This is the control force computed based on the given control command
#     # If using force control, it's the same as the given control command
#     print("control force:", franka.get_dofs_control_force(dofs_idx))

#     # This is the actual force experienced by the dof
#     print("internal force:", franka.get_dofs_force(dofs_idx))

    scene.step()

#!/usr/bin/env python
import io

from cached_property import cached_property
import numpy as np
import six

from matrix import get_rotation_matrix, get_transform_matrix


class Kinematics(object):
    """Compute kineamtics for each part, e.g. left-leg
    """
    def __init__(self, link_list, join_list, link_const):
        self.link_list = link_list
        self.joint_list = joint_list
        self.const_list = const_list

    def forward_kinematics(self, joint_angles):
        """Returns calculated position of end-effector

        Args:
            joint_angles (list): Target joint angles for each joint

        Returns:
            pos (np.ndarray): in shape (3,)
        """
        T_ = np.ones((3, 4))
        for joint, angle in zip(self.joint_list, joint_angles):
            T_ *= self._calc_tf_matrix(angle, joint)

        return T_[:3, -1]

    def inverse_kinematics(self, pos, reverse):
        return

    def _inverse_kinematics_loop(self):
        return

    def _calc_tf_matrix(self, angle, joint):
        alpha = 0
        if "hip" in joint.name:
            alpha = np.pi / 2
        tf = get_transform_matrix(angle, joint, alpha)
        return tf


class RobotModel(Kinematics):
    def __init__(self, link_list=None, join_list=None):
        super(RobotModel, self).__init__(link_list, join_list)

        self.joint_names = []
        for joint, link in zip(self.joint_list, self.link_list):
            self.joint_names.append(joint.name)
            self.__dict__[link.name] = link
            self.__dict__[joint.name] = joint

        self.urdf_path = None

    @property
    def init_pose(self):
        raise NotImplementedError()

    def _mehses_from_urdf_visuals(self, visuals):
        meshes = []
        for visual in visuals:
            meshes.extend(self._meshes_from_urdf_visual(visual))
        return meshes

    def _meshes_from_urdf_visual(self, visual):
        if not isinstance(visual, urdf.Visual):
            raise TypeError("visual must be urdf.Visual, but got: {}".format(type(visual)))

        meshes = []
        for mesh in visual.geometry.meshes:
            mesh = mesh.copy()

            # rescale
            if visual.geometry.mesh is not None:
                if visual.geometry.mesh.scale is not None:
                    mesh.vertices = mesh.vertices * visual.geometry.mesh.scale

            # TextureVisuals is usually slow to render
            if not isinstance(mesh.visual, trimesh.visual.ColorVisuals):
                mesh.visual = mesh.visual.to_color()
                if mesh.visual.vertex_colors.ndim == 1:
                    mesh.visual.vertex_colors = \
                        mesh.visual.vertex_colors[None].repeat(
                            mesh.vertices.shape[0], axis=0
                        )

            # If color or texture is not specified in mesh file,
            # use information specified in URDF.
            if (
                (mesh.visual.face_colors
                 == trimesh.visual.DEFAULT_COLOR).all()
                and visual.material
            ):
                if visual.material.texture is not None:
                    warnings.warn(
                        'texture specified in URDF is not supported'
                    )
                elif visual.material.color is not None:
                    mesh.visual.face_colors = visual.material.color

            mesh.apply_transform(visual.origin)
            mesh.metadata["origin"] = visual.origin
            meshes.append(mesh)
        return meshes

    def parse_cfg(self, cfg):
        """Parse model config.cfg and returns model config as dict"""
        return parse_cfg(cfg)

    def load_urdf(self, urdf):
        return

    def load_urdf_file(self, file_obj):
        return


class RobotModelFromURDF(RobotModel):
    def __init__(self, urdf=None, urdf_file=None):
        if urdf and urdf_file:
            raise ValueError(
                "'urdf' and 'urdf_file' cannot be given at the same time"
            )

        if urdf:
            self.load_urdf(urdf=urdf)
        elif urdf_file:
            self.load_urdf_file(file_obj=urdf_file)
        else:
            self.load_urdf_file(file_obj=self.default_urdf_path)

    def load_urdf(self, urdf):
        return

    def load_urdf_file(self, file_obj):
        return

    @property
    def default_urdf_path(self):
        raise NotImplementedError()


class Spot(RobotModelFromURDF):
    def __init__(self, *args, **kwargs):
        super(Spot, self).__init__(*args, **kwargs)
        

    @cached_property
    def lfoot(self):
        link_list = []
        joint_list = []
        r = RobotModel(link_list, joint_list)
        return r

    @cached_property
    def lhip(self):
        link_list = []
        joint_list = []
        r = RobotModel(link_list, joint_list)
        return r

    @cached_property
    def rfoot(self):
        link_list = []
        joint_list = []
        r = RobotModel(link_list, joint_list)
        return r

    @cached_property
    def rhip(self):
        link_list = []
        joint_list = []
        r = RobotModel(link_list, joint_list)
        return r

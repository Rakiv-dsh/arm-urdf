"""
MuJoCo-based robot wrapper for kinematics computation.
Provides the same interface as RobotWrapper but uses MuJoCo instead of Pinocchio.
Supports loading from XML string for dynamic model updates.
"""
import os
import re
from pathlib import Path
from typing import List, Optional, Union, Dict

import numpy as np
import numpy.typing as npt
import mujoco


class MujocoRobotWrapper:
    """
    Robot wrapper using MuJoCo for kinematics computation.
    This class does not take mimic joint into consideration.
    """

    # 类级别的asset缓存，避免重复加载相同的文件
    _asset_cache: Dict[str, bytes] = {}
    _verbose_assets: bool = True

    @classmethod
    def set_verbose_assets(cls, verbose: bool):
        """设置是否打印asset加载信息"""
        cls._verbose_assets = verbose

    @classmethod
    def clear_asset_cache(cls):
        """清空asset缓存"""
        cls._asset_cache.clear()

    def __init__(
        self,
        xml_source: str,
        from_string: bool = True,
        assets_dir: Optional[str] = None,
        use_cached_assets: bool = True,
        skip_mesh_loading: bool = False,
    ):
        """
        Initialize the MuJoCo robot wrapper.

        Args:
            xml_source: Either XML string content or path to XML file
            from_string: If True, xml_source is treated as XML string content.
                        If False, xml_source is treated as file path.
            assets_dir: Directory containing mesh files and other assets.
                       Required when from_string=True and XML contains relative paths.
            use_cached_assets: If True, use cached assets for repeated loads.
            skip_mesh_loading: If True, skip mesh file loading (use custom collision).
        """
        if from_string and skip_mesh_loading:
            # 跳过mesh加载，只使用XML几何体
            self.model: mujoco.MjModel = mujoco.MjModel.from_xml_string(xml_source, assets=None)
        elif from_string:
            # Load assets if assets_dir is provided
            assets = None
            if assets_dir is not None:
                assets = self._load_assets(xml_source, assets_dir, use_cache=use_cached_assets)
            self.model: mujoco.MjModel = mujoco.MjModel.from_xml_string(xml_source, assets=assets)
        else:
            self.model: mujoco.MjModel = mujoco.MjModel.from_xml_path(xml_source)

        self.data: mujoco.MjData = mujoco.MjData(self.model)

        # Cache link (body) names and joint names
        self._link_names = self._get_body_names()
        self._joint_names = self._get_joint_names()
        self._dof_joint_names = self._get_dof_joint_names()

        # Initial configuration
        self.q0 = np.zeros(self.model.nq)

        if self.model.nv != self.model.nq:
            raise NotImplementedError(f"Cannot handle robot with special joint. nv={self.model.nv}, nq={self.model.nq}")

    def _load_assets(self, xml_string: str, assets_dir: str, use_cache: bool = True) -> Dict[str, bytes]:
        """
        Load mesh files and other assets referenced in the XML.

        Args:
            xml_string: XML content as string
            assets_dir: Base directory for resolving relative paths
            use_cache: If True, use cached assets when available

        Returns:
            Dictionary mapping file paths to file contents
        """
        assets = {}
        assets_path = Path(assets_dir)

        # Find all file references in the XML (mesh files, textures, etc.)
        # Pattern matches: file="..." or filename="..."
        file_pattern = re.compile(r'file\s*=\s*["\']([^"\']+)["\']')

        for match in file_pattern.finditer(xml_string):
            file_ref = match.group(1)

            # Resolve the file path
            if file_ref.startswith('/'):
                # Absolute path
                file_path = Path(file_ref)
            else:
                # Relative path - resolve from assets_dir and normalize
                file_path = (assets_path / file_ref).resolve()

            # Use cache if available and enabled
            if use_cache and file_ref in self._asset_cache:
                assets[file_ref] = self._asset_cache[file_ref]
                if self._verbose_assets:
                    print(f"  [Cached] {file_ref}")
            # Load from disk if not cached
            elif file_path.exists():
                with open(file_path, 'rb') as f:
                    content = f.read()
                    assets[file_ref] = content
                    # Cache for future use
                    if use_cache:
                        self._asset_cache[file_ref] = content
                    if self._verbose_assets:
                        print(f"  Loaded asset: {file_ref} -> {file_path}")

        return assets

    def _get_body_names(self) -> List[str]:
        """Get all body names in the model."""
        names = []
        for i in range(self.model.nbody):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, i)
            if name:
                names.append(name)
        return names

    def _get_joint_names(self) -> List[str]:
        """Get all joint names in the model."""
        names = []
        for i in range(self.model.njnt):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
            if name:
                names.append(name)
        return names

    def _get_dof_joint_names(self) -> List[str]:
        """Get joint names that have degrees of freedom (exclude fixed joints)."""
        names = []
        for i in range(self.model.njnt):
            # Check if joint has DOF (not a fixed joint)
            jnt_type = self.model.jnt_type[i]
            if jnt_type != mujoco.mjtJoint.mjJNT_FREE:  # All non-free joints have 1 DOF in our case
                name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
                if name:
                    names.append(name)
        return names

    # -------------------------------------------------------------------------- #
    # Robot property
    # -------------------------------------------------------------------------- #
    @property
    def joint_names(self) -> List[str]:
        """Return all joint names."""
        return self._joint_names

    @property
    def dof_joint_names(self) -> List[str]:
        """Return joint names that have degrees of freedom."""
        return self._dof_joint_names

    @property
    def dof(self) -> int:
        """Return the number of degrees of freedom."""
        return self.model.nq

    @property
    def link_names(self) -> List[str]:
        """Return all link (body) names."""
        return self._link_names

    @property
    def joint_limits(self) -> npt.NDArray:
        """Return joint limits as (n, 2) array with [lower, upper] bounds."""
        lower = self.model.jnt_range[:, 0].copy()
        upper = self.model.jnt_range[:, 1].copy()
        return np.stack([lower, upper], axis=1)

    # -------------------------------------------------------------------------- #
    # Query function
    # -------------------------------------------------------------------------- #
    def get_joint_index(self, name: str) -> int:
        """Get the index of a joint by name."""
        return self.dof_joint_names.index(name)

    def get_link_index(self, name: str) -> int:
        """Get the index of a link (body) by name."""
        if name not in self._link_names:
            raise ValueError(f"{name} is not a link name. Valid link names: \n{self._link_names}")
        return mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)

    def get_joint_parent_child_frames(self, joint_name: str):
        """Get parent and child body IDs for a joint."""
        joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        if joint_id < 0:
            raise ValueError(f"Joint {joint_name} not found")

        # In MuJoCo, joint belongs to a body (child), and parent is body's parent
        child_body_id = self.model.jnt_bodyid[joint_id]
        parent_body_id = self.model.body_parentid[child_body_id]

        return parent_body_id, child_body_id

    # -------------------------------------------------------------------------- #
    # Kinematics function
    # -------------------------------------------------------------------------- #
    def compute_forward_kinematics(self, qpos: npt.NDArray):
        """
        Compute forward kinematics for the given joint positions.

        Args:
            qpos: Joint positions array of shape (nq,)
        """
        self.data.qpos[:] = qpos
        self.data.qvel[:] = 0  # 设置速度为 0
        mujoco.mj_forward(self.model, self.data)  # 使用 mj_forward 完全更新状态

    def get_link_pose(self, link_id: int) -> npt.NDArray:
        """
        Get the pose of a link as a 4x4 homogeneous transformation matrix.
        For fingertip bodies, returns the tip geom position instead of body origin.

        Args:
            link_id: Body ID of the link

        Returns:
            4x4 homogeneous transformation matrix
        """
        rot_mat = self.data.xmat[link_id].reshape(3, 3).copy()

        # Check if this body has a tip geom (for fingertips)
        body_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, link_id)
        tip_geom_name = None

        # Map body names to their tip geom names
        tip_geom_map = {
            "if_ds": "if_tip",
            "mf_ds": "mf_tip",
            "rf_ds": "rf_tip",
            "th_ds": "th_tip",
        }

        if body_name in tip_geom_map:
            tip_geom_name = tip_geom_map[body_name]
            tip_geom_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, tip_geom_name)
            if tip_geom_id >= 0:
                # Use tip geom position
                pos = self.data.geom_xpos[tip_geom_id].copy()
            else:
                pos = self.data.xpos[link_id].copy()
        else:
            pos = self.data.xpos[link_id].copy()

        pose = np.eye(4)
        pose[:3, :3] = rot_mat
        pose[:3, 3] = pos
        return pose

    def get_link_pose_inv(self, link_id: int) -> npt.NDArray:
        """
        Get the inverse pose of a link as a 4x4 homogeneous transformation matrix.

        Args:
            link_id: Body ID of the link

        Returns:
            4x4 inverse homogeneous transformation matrix
        """
        pose = self.get_link_pose(link_id)
        pose_inv = np.eye(4)
        pose_inv[:3, :3] = pose[:3, :3].T
        pose_inv[:3, 3] = -pose[:3, :3].T @ pose[:3, 3]
        return pose_inv

    def compute_single_link_local_jacobian(self, qpos: npt.NDArray, link_id: int) -> npt.NDArray:
        """
        Compute the Jacobian for a single link in the local (body) frame.
        For fingertip bodies, computes Jacobian at the tip geom position.

        Note: This assumes compute_forward_kinematics has already been called with the same qpos.

        Args:
            qpos: Joint positions array of shape (nq,) - used for consistency check
            link_id: Body ID of the link

        Returns:
            6 x nv Jacobian matrix (3 rows for position, 3 rows for rotation)
        """
        # Allocate Jacobian matrices
        jacp = np.zeros((3, self.model.nv))  # Position Jacobian
        jacr = np.zeros((3, self.model.nv))  # Rotation Jacobian

        # Check if this body has a tip geom (for fingertips)
        body_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, link_id)

        # Map body names to their tip geom names
        tip_geom_map = {
            "if_ds": "if_tip",
            "mf_ds": "mf_tip",
            "rf_ds": "rf_tip",
            "th_ds": "th_tip",
        }

        if body_name in tip_geom_map:
            tip_geom_name = tip_geom_map[body_name]
            tip_geom_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, tip_geom_name)
            if tip_geom_id >= 0:
                # Use tip geom position for Jacobian computation
                point = self.data.geom_xpos[tip_geom_id].copy()
            else:
                point = self.data.xpos[link_id].copy()
        else:
            point = self.data.xpos[link_id].copy()

        mujoco.mj_jac(self.model, self.data, jacp, jacr, point, link_id)

        # Stack position and rotation Jacobians
        J = np.vstack([jacp, jacr])

        return J

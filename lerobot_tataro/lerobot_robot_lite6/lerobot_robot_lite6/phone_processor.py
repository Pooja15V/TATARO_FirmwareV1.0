"""Lite6WebXRPhone action (robot axes) -> Lite6 absolute EE action, with a clutch.

Pipeline (see scripts/tataro_common.py):
    Lite6WebXRPhone.get_action() -> Lite6PhoneToEE (this file) -> Lite6.send_action()

While Move is held the TCP follows the phone's motion relative to the moment Move was pressed
(position, and orientation unless use_rotation=False). Release -> arm holds still.
"""

from dataclasses import dataclass, field

import numpy as np
from scipy.spatial.transform import Rotation

from lerobot.configs import FeatureType, PipelineFeatureType, PolicyFeature
from lerobot.lerobot_types import RobotAction
from lerobot.processor import ProcessorStepRegistry, RobotActionProcessorStep, TransitionKey

from .pose_utils import EE_KEYS, dict_to_pR, pR_to_dict

_IN_KEYS = ["phone.enabled", "phone.pos", "phone.rot", "phone.gripper"]


@ProcessorStepRegistry.register("lite6_phone_to_ee")
@dataclass
class Lite6PhoneToEE(RobotActionProcessorStep):
    position_scale: float = 1.0  # metres of TCP motion per metre of phone motion
    use_rotation: bool = True  # False -> keep orientation fixed, translate only

    _prev_enabled: bool = field(default=False, init=False, repr=False)
    _ref_p: np.ndarray | None = field(default=None, init=False, repr=False)
    _ref_R: Rotation | None = field(default=None, init=False, repr=False)
    _edge_p: np.ndarray | None = field(default=None, init=False, repr=False)
    _edge_R: Rotation | None = field(default=None, init=False, repr=False)
    _hold: tuple | None = field(default=None, init=False, repr=False)
    _grip: float | None = field(default=None, init=False, repr=False)

    def action(self, action: RobotAction) -> RobotAction:
        obs = self.transition.get(TransitionKey.OBSERVATION)
        if obs is None:
            raise ValueError("Lite6PhoneToEE needs the robot observation (ee.* keys)")
        p_cur, R_cur = dict_to_pR(obs)
        if self._grip is None:
            self._grip = float(obs.get("gripper.pos", 0.0))

        enabled = bool(action.pop("phone.enabled"))
        p_ph = np.asarray(action.pop("phone.pos"), dtype=float)
        R_ph = action.pop("phone.rot")
        grip = action.pop("phone.gripper", None)

        if enabled:
            if not self._prev_enabled or self._ref_p is None:
                self._ref_p, self._ref_R = p_cur.copy(), R_cur
                self._edge_p, self._edge_R = p_ph.copy(), R_ph
            p_des = self._ref_p + self.position_scale * (p_ph - self._edge_p)
            # phone's orientation change since Move was pressed, applied in the base frame
            R_des = (R_ph * self._edge_R.inv()) * self._ref_R if self.use_rotation else self._ref_R
            self._hold = (p_des, R_des)
        elif self._hold is None:
            self._hold = (p_cur.copy(), R_cur)
        self._prev_enabled = enabled

        if grip is not None:
            self._grip = float(grip)

        action.update(pR_to_dict(*self._hold))
        action["gripper.pos"] = self._grip
        return action

    def reset(self) -> None:
        self._prev_enabled = False
        self._ref_p = self._ref_R = self._edge_p = self._edge_R = None
        self._hold = None
        self._grip = None

    def transform_features(
        self, features: dict[PipelineFeatureType, dict[str, PolicyFeature]]
    ) -> dict[PipelineFeatureType, dict[str, PolicyFeature]]:
        for k in _IN_KEYS:
            features[PipelineFeatureType.ACTION].pop(k, None)
        for k in (*EE_KEYS, "gripper.pos"):
            features[PipelineFeatureType.ACTION][k] = PolicyFeature(type=FeatureType.ACTION, shape=(1,))
        return features

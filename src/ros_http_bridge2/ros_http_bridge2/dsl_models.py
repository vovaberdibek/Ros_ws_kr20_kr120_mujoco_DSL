from typing import List, Optional

from pydantic import BaseModel, Field, validator


class ScrewModel(BaseModel):
    quantity: int = Field(..., ge=0)
    type: str


class UnitScrewSpec(BaseModel):
    positions: Optional[List[List[float]]] = None
    manual_indices: Optional[List[int]] = None
    auto_indices: Optional[List[int]] = None

    @validator("positions", each_item=True)
    def check_pose_len(cls, pose: List[float]) -> List[float]:
        if pose is not None and len(pose) != 6:
            raise ValueError("each screw pose must contain 6 values")
        return pose


class UnitModel(BaseModel):
    name: str
    pose_index: Optional[int] = None
    screws: Optional[UnitScrewSpec] = None

    class Config:
        extra = "allow"


class TrayModel(BaseModel):
    name: str
    tray: Optional[str]
    units: List[UnitModel] = Field(default_factory=list)
    screws: List[ScrewModel] = Field(default_factory=list)
    height: Optional[float]
    initial_pose: Optional[List[float]]
    operator_pose: Optional[List[float]]
    final_pose: Optional[List[float]]

    @validator("initial_pose", "operator_pose", "final_pose")
    def pose_must_have_six_values(cls, v: Optional[List[float]]) -> Optional[List[float]]:
        if v is None:
            return v
        if len(v) != 6:
            raise ValueError("pose must contain exactly 6 values [x,y,z,rx,ry,rz]")
        return v


class DSLValidationError(RuntimeError):
    """Custom error raised when DSL semantic validation fails."""

    def __init__(self, errors: List[str]):
        super().__init__("\n".join(errors))

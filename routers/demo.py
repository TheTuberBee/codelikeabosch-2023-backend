from fastapi import APIRouter, Depends

from typing import Annotated

from dependencies.file import parse_dataset, Tick
from common.world import World, Object, WorldSnapshot


router = APIRouter()


@router.post("/")
async def upload_demo(
    ticks: Annotated[list[Tick], Depends(parse_dataset)]
):
    world = World()

    snapshots: list[WorldSnapshot] = []
    for tick in ticks:
        snapshot = world.tick(tick)
        snapshots.append(snapshot)

    return snapshots

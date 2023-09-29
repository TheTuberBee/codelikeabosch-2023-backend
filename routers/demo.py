from fastapi import APIRouter, Depends

from typing import Annotated

from dependencies.file import parse_dataset, Tick
from common.world import World, Object, WorldSnapshot


router = APIRouter()


@router.post("/")
async def upload_demo(
    ticks: Annotated[list[Tick], Depends(parse_dataset)]
):
    world = World(ticks[0])

    snapshots: list[WorldSnapshot] = []
    for tick in ticks[1:]:
        snapshot = world.tick(tick)
        snapshots.append(snapshot)

    return snapshots

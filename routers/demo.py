from fastapi import APIRouter, Depends

from typing import Annotated

from dependencies.file import parse_dataset, Tick
from common.world import World, Object


router = APIRouter()


@router.post("/")
async def upload_demo(
    ticks: Annotated[list[Tick], Depends(parse_dataset)]
):
    world = World()

    for tick in ticks:
        world.tick(tick)


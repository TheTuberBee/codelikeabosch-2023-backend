from fastapi import APIRouter, Depends

from typing import Annotated

from dependencies.file import parse_dataset, get_filename
from common.world import World, WorldSnapshot, Tick

from models import Demo

router = APIRouter()


@router.post("/")
async def upload_demo(
    ticks: Annotated[list[Tick], Depends(parse_dataset)],
    filename: Annotated[str, Depends(get_filename)],
):
    world = World(ticks[0])

    snapshots: list[WorldSnapshot] = []
    for tick in ticks[1:]:
        snapshot = world.tick(tick)
        snapshots.append(snapshot)

    existing_demo = await Demo.filter(name=filename).first()
    if existing_demo:
        filename = "new_" + filename

    await Demo.create(
        name=filename,
        data=[snapshot.dict() for snapshot in snapshots]
    )

    return snapshots

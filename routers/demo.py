from fastapi import APIRouter, Depends, HTTPException

from typing import Annotated

from dependencies.file import parse_dataset, get_filename
from common.world import World, WorldSnapshot, Tick
from time import time
from models import Demo

router = APIRouter()


@router.post("")
async def upload_demo(
    ticks: Annotated[list[Tick], Depends(parse_dataset)],
    filename: Annotated[str, Depends(get_filename)],
):
    world = World(ticks[0])

    snapshots: list[WorldSnapshot] = []
    for tick in ticks[1:]:
        snapshot = world.tick(tick)
        snapshots.append(snapshot)

    await Demo.create(
        name=filename + "_" + str(int(time())),
        data=[snapshot.dict() for snapshot in snapshots]
    )

    return snapshots


@router.get("")
async def get_demos():
    records = await Demo.all().values("id", "name")
    return records


@router.get("/{demo_id}")
async def get_demo(demo_id: int):
    demo = await Demo.get(id=demo_id)
    return demo.data
